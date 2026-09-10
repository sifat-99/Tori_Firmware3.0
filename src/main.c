#include "driver/gpio.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_cam_sensor.h"
#include "esp_ldo_regulator.h"
#include "esp_video_device.h"
#include "esp_video_init.h"

#include "driver/jpeg_encode.h"
#include "esp_http_server.h"
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

static const char *TAG = "eth_cam";

#define CAM_SCCB_I2C_PORT 0
#define CAM_SCCB_SCL_PIN 8
#define CAM_SCCB_SDA_PIN 7
#define CAM_SCCB_FREQ_HZ 100000
#define CAM_RESET_PIN (44)
#define CAM_PWDN_PIN (43)
#define CAM_DEV_PATH ESP_VIDEO_MIPI_CSI_DEVICE_NAME

static const esp_video_init_csi_config_t csi_config[] = {{
    .sccb_config =
        {
            .init_sccb = true,
            .i2c_config = {.port = CAM_SCCB_I2C_PORT,
                           .scl_pin = CAM_SCCB_SCL_PIN,
                           .sda_pin = CAM_SCCB_SDA_PIN},
            .freq = CAM_SCCB_FREQ_HZ,
        },
    .reset_pin = CAM_RESET_PIN,
    .pwdn_pin = CAM_PWDN_PIN,
}};

static const esp_video_init_config_t cam_config = {
    .csi = csi_config,
};

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;

  int fd = open(CAM_DEV_PATH, O_RDONLY);
  if (fd < 0) {
    ESP_LOGE(TAG, "open %s failed", CAM_DEV_PATH);
    return ESP_FAIL;
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  struct v4l2_format fmt = {.type = type};
  ioctl(fd, VIDIOC_G_FMT, &fmt);

  // We request RGB565 because hardware JPEG encoder on v1.3 does not support
  // YUV420
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
  ioctl(fd, VIDIOC_S_FMT, &fmt);
  ioctl(fd, VIDIOC_G_FMT, &fmt);

  uint32_t w = fmt.fmt.pix.width, h = fmt.fmt.pix.height;
  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
    ESP_LOGE(TAG, "Expected RGB565 from ISP, got %x", fmt.fmt.pix.pixelformat);
    close(fd);
    return ESP_FAIL;
  }

  struct v4l2_requestbuffers reqbuf = {
      .count = 2, .type = type, .memory = V4L2_MEMORY_MMAP};
  ioctl(fd, VIDIOC_REQBUFS, &reqbuf);

  // --- Fix Green Tint by Manually Setting White Balance Gains ---
  // AWB might not be running in the background, so we set manual multipliers.
  // The denominator is 1000. Setting to 2000 means 2.0x gain for Red and Blue.
  struct v4l2_ext_control c_red = {.id = V4L2_CID_RED_BALANCE, .value = 2000};
  struct v4l2_ext_controls cs_red = {
      .ctrl_class = V4L2_CTRL_CLASS_USER, .count = 1, .controls = &c_red};
  ioctl(fd, VIDIOC_S_EXT_CTRLS, &cs_red);

  struct v4l2_ext_control c_blue = {.id = V4L2_CID_BLUE_BALANCE, .value = 2000};
  struct v4l2_ext_controls cs_blue = {
      .ctrl_class = V4L2_CTRL_CLASS_USER, .count = 1, .controls = &c_blue};
  ioctl(fd, VIDIOC_S_EXT_CTRLS, &cs_blue);

  ESP_LOGI(TAG, "Manual White Balance Applied: R=2.0x, B=2.0x");

  // --- Maximize Brightness by Manually Setting Max Exposure and Gain ---
  struct v4l2_query_ext_ctrl q_exp = {.id = V4L2_CID_EXPOSURE};
  if (ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q_exp) == 0) {
    struct v4l2_ext_control c_exp = {.id = V4L2_CID_EXPOSURE,
                                     .value =
                                         q_exp.maximum}; // 100% of max exposure
    struct v4l2_ext_controls cs_exp = {
        .ctrl_class = V4L2_CTRL_CLASS_USER, .count = 1, .controls = &c_exp};
    ioctl(fd, VIDIOC_S_EXT_CTRLS, &cs_exp);
    ESP_LOGI(TAG, "Manual Exposure set to MAXIMUM (%lld)",
             (long long)q_exp.maximum);
  }

  struct v4l2_querymenu q_gain = {.id = V4L2_CID_GAIN, .index = 10};
  int max_gain_idx = 0;
  while (ioctl(fd, VIDIOC_QUERYMENU, &q_gain) == 0) {
    max_gain_idx = q_gain.index;
    q_gain.index++;
  }
  if (max_gain_idx > 0) {
    // Let's set it to maximum possible analog gain for brightness!
    struct v4l2_ext_control c_gain = {.id = V4L2_CID_GAIN,
                                      .value = max_gain_idx};
    struct v4l2_ext_controls cs_gain = {
        .ctrl_class = V4L2_CTRL_CLASS_USER, .count = 1, .controls = &c_gain};
    ioctl(fd, VIDIOC_S_EXT_CTRLS, &cs_gain);
    ESP_LOGI(TAG, "Manual Gain set to MAXIMUM (%d)", max_gain_idx);
  }
  // --------------------------------------------------------

  uint8_t *buffer[2];
  for (int i = 0; i < 2; i++) {
    struct v4l2_buffer b = {
        .type = type, .memory = V4L2_MEMORY_MMAP, .index = i};
    ioctl(fd, VIDIOC_QUERYBUF, &b);
    buffer[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                     b.m.offset);
    ioctl(fd, VIDIOC_QBUF, &b);
  }

  ioctl(fd, VIDIOC_STREAMON, &type);

  jpeg_encoder_handle_t enc = NULL;
  jpeg_encode_engine_cfg_t eng = {.timeout_ms = 5000};
  if (jpeg_new_encoder_engine(&eng, &enc) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create JPEG engine");
    close(fd);
    return ESP_FAIL;
  }

  size_t out_alloc = 0;
  jpeg_encode_memory_alloc_cfg_t mem = {.buffer_direction =
                                            JPEG_ENC_ALLOC_OUTPUT_BUFFER};
  uint8_t *out = jpeg_alloc_encoder_mem(w * h, &mem, &out_alloc);

  jpeg_encode_cfg_t cfg = {
      .width = w,
      .height = h,
      .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
      .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
      .image_quality = 60,
  };

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    close(fd);
    return res;
  }

  char part_buf[64];
  while (true) {
    struct v4l2_buffer buf = {.type = type, .memory = V4L2_MEMORY_MMAP};
    if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) {
      ESP_LOGE(TAG, "DQBUF failed");
      break;
    }

    uint32_t out_len = 0;
    esp_err_t enc_ret = jpeg_encoder_process(
        enc, &cfg, buffer[buf.index], w * h * 2, out, out_alloc, &out_len);

    ioctl(fd, VIDIOC_QBUF, &buf);

    if (enc_ret != ESP_OK) {
      ESP_LOGE(TAG, "JPEG encode failed: %d", enc_ret);
      continue;
    }

    size_t hlen = snprintf(part_buf, 64, _STREAM_PART, out_len);
    res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)out, out_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY,
                                  strlen(_STREAM_BOUNDARY));
    }
    if (res != ESP_OK) {
      break;
    }
  }

  ioctl(fd, VIDIOC_STREAMOFF, &type);
  close(fd);
  jpeg_del_encoder_engine(enc);
  if (out)
    free(out);

  return res;
}

// -----------------------------------------------------------------
// OPTIONS Handler added to respond properly to Preflight requests
// from Chrome/Web browsers to bypass CORS security constraints.
// -----------------------------------------------------------------
static esp_err_t options_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers",
                     "Content-Type, Authorization");
  httpd_resp_set_status(req, "204 No Content");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static void start_camera_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t stream_uri = {.uri = "/stream",
                            .method = HTTP_GET,
                            .handler = stream_handler,
                            .user_ctx = NULL};

  httpd_uri_t stream_options_uri = {.uri = "/stream",
                                    .method = HTTP_OPTIONS,
                                    .handler = options_handler,
                                    .user_ctx = NULL};

  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &stream_uri);
    httpd_register_uri_handler(server, &stream_options_uri);
    ESP_LOGI(TAG, "HTTP Server started on port 80. Path: /stream");
  }
}

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data) {
  uint8_t mac_addr[6] = {0};
  esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

  switch (event_id) {
  case ETHERNET_EVENT_CONNECTED:
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
    ESP_LOGI(TAG, "Ethernet Link Up");
    ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0],
             mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    break;
  case ETHERNET_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "Ethernet Link Down");
    break;
  case ETHERNET_EVENT_START:
    ESP_LOGI(TAG, "Ethernet Started");
    break;
  case ETHERNET_EVENT_STOP:
    ESP_LOGI(TAG, "Ethernet Stopped");
    break;
  default:
    break;
  }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data) {
  ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
  const esp_netif_ip_info_t *ip_info = &event->ip_info;
  ESP_LOGI(TAG, "Ethernet Got IP Address");
  ESP_LOGI(TAG, "~~~~~~~~~~~");
  ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
  ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
  ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
  ESP_LOGI(TAG, "~~~~~~~~~~~");
}

void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Initialize Ethernet MAC and PHY (IP101)
  esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
  esp_netif_t *eth_netif = esp_netif_new(&cfg);

  // Set Static IP for local direct connection
  esp_netif_dhcpc_stop(eth_netif);
  esp_netif_ip_info_t ip_info;
  memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));
  esp_netif_str_to_ip4("192.168.1.100", &ip_info.ip);
  esp_netif_str_to_ip4("192.168.1.1", &ip_info.gw);
  esp_netif_str_to_ip4("255.255.255.0", &ip_info.netmask);
  esp_netif_set_ip_info(eth_netif, &ip_info);

  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                             &eth_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                             &got_ip_event_handler, NULL));

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
  esp32_emac_config.smi_gpio.mdc_num = 31;
  esp32_emac_config.smi_gpio.mdio_num = 52;

  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = 1;
  phy_config.reset_gpio_num = 51; // Reset pin based on board documentation

  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
  esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
  esp_eth_handle_t eth_handle = NULL;
  ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
  ESP_ERROR_CHECK(
      esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
  ESP_ERROR_CHECK(esp_eth_start(eth_handle));

  ESP_LOGI(TAG, "Ethernet initialized successfully. Waiting for IP...");

  // Initialize Camera
  if (esp_video_init(&cam_config) != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_init failed");
  } else {
    ESP_LOGI(TAG, "Camera Initialized successfully.");
    start_camera_server();
  }

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
