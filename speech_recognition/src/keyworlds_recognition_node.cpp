#include <errno.h>

#include "rclcpp/rclcpp.hpp"

#include <speechapi_cxx.h>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <future>
#include <iostream>  // cin, cout
#include <regex>  
#include <string>

#include "speechapi_c_ext_audiocompression.h"

// #include <alsa/asoundlib.h>

using namespace std;
using namespace Microsoft::CognitiveServices::Speech;
using namespace Microsoft::CognitiveServices::Speech::Audio;

int Microsoft_CognitiveServices = 0;

extern "C" {
#include "msp_cmn.h"
#include "msp_errors.h"
#include "qisr.h"
#include "speech_recognizer.h"
}

#define FRAME_LEN 640
#define BUFFER_SIZE 4096
#define SAMPLE_RATE_16K (16000)
#define SAMPLE_RATE_8K (8000)
#define MAX_GRAMMARID_LEN (32)
#define MAX_PARAMS_LEN (1024)

const char *ASR_RES_PATH = "fo|res/asr/common.jet";  //离线语法识别资源路径
const char *GRM_BUILD_PATH =
    "res/asr/GrmBuilld";  //构建离线语法识别网络生成数据保存路径
const char *GRM_FILE = "call.bnf";  //构建离线识别语法网络所用的语法文件
const char *LEX_NAME =
    "contact";  //更新离线识别语法的contact槽（语法文件为此示例中使用的call.bnf）

typedef struct _UserData {
  int build_fini;   //标识语法构建是否完成
  int update_fini;  //标识更新词典是否完成
  int errcode;      //记录语法构建或更新词典回调错误码
  char grammar_id[MAX_GRAMMARID_LEN];  //保存语法构建返回的语法ID
} UserData;

static void show_result(char *string, char is_over) {
  printf("\rResult: [ %s ]", string);
  if (is_over) putchar('\n');
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;


void processSpeechRecognizer() {
  std::string file = "output.wav";
  auto audioConfig = AudioConfig::FromWavFileInput(file);
  setlocale(LC_ALL, "");
  auto config = SpeechConfig::FromSubscription(
      "5dcf6f4b26c348efa1164faa5ab432da", "australiaeast");
  config->SetSpeechRecognitionLanguage("zh-CN");
  auto speechRecognizer = SpeechRecognizer::FromConfig(config, audioConfig);
  cout << "Say something...\n";
  auto result = speechRecognizer->RecognizeOnceAsync().get();
  // Checks result.
  cout << "RECOGNIZED: Text=" << result->Text << std::endl;
}

void on_result(const char *result, char is_last) {
  if (result) {
    size_t left = g_buffersize - 1 - strlen(g_result);
    size_t size = strlen(result);
    if (left < size) {
      g_result = (char *)realloc(g_result, g_buffersize + BUFFER_SIZE);
      if (g_result)
        g_buffersize += BUFFER_SIZE;
      else {
        printf("mem alloc failed\n");
        return;
      }
    }
    strncat(g_result, result, size);
    show_result(g_result, is_last);
  }
}
void on_speech_begin() {
  if (g_result) {
    free(g_result);
  }
  g_result = (char *)malloc(BUFFER_SIZE);
  g_buffersize = BUFFER_SIZE;
  memset(g_result, 0, g_buffersize);

  printf("Start Listening...\n");
}
void on_speech_end(int reason) {
  if (reason == END_REASON_VAD_DETECT) {
    printf("\nSpeaking done \n");
    Microsoft_CognitiveServices = 1;

  } else
    printf("\nRecognizer error %d\n", reason);
}

static void demo_mic(const char *session_begin_params) {
  int errcode;
  int i = 0;

  struct speech_rec iat;

  struct speech_rec_notifier recnotifier = {on_result, on_speech_begin,
                                            on_speech_end};

  errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
  if (errcode) {
    printf("speech recognizer init failed\n");
    return;
  }
  errcode = sr_start_listening(&iat);
  if (errcode) {
    printf("start listen failed %d\n", errcode);
  }
  /* demo 15 seconds recording */
  while (i++ < 15) {
    if (iat.ep_stat == MSP_EP_AFTER_SPEECH) {
      break;
    }
    sleep(1);
  }
  errcode = sr_stop_listening(&iat);
  if (errcode) {
    printf("stop listening failed %d\n", errcode);
  }

  sr_uninit(&iat);
}

int run_asr(UserData *udata) {
  char asr_params[MAX_PARAMS_LEN] = {NULL};
  const char *rec_rslt = NULL;
  const char *session_id = NULL;
  const char *asr_audiof = NULL;
  FILE *f_pcm = NULL;
  char *pcm_data = NULL;
  long pcm_count = 0;
  long pcm_size = 0;
  int last_audio = 0;

  int aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;
  int ep_status = MSP_EP_LOOKING_FOR_SPEECH;
  int rec_status = MSP_REC_STATUS_INCOMPLETE;
  int rss_status = MSP_REC_STATUS_INCOMPLETE;
  int errcode = -1;
  int aud_src = 0;
  //离线语法识别参数设置
  snprintf(asr_params, MAX_PARAMS_LEN - 1,
           "engine_type = local, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, local_grammar = %s, \
		result_type = xml, result_encoding = UTF-8, \
		vad_eos = 900, asr_denoise = 1, vad_enable = 1",
           ASR_RES_PATH, SAMPLE_RATE_16K, GRM_BUILD_PATH, udata->grammar_id);
  demo_mic(asr_params);

  return 0;
}



void awake_callback(const std_msgs::msg::Int32 & msg)
{
  // RCLCPP_INFO("I heard: '");
  int ret = 0;
  UserData asr_data;
  memset(&asr_data, 0, sizeof(UserData));
  strcpy(asr_data.grammar_id, "call");
  ret = run_asr(&asr_data);
  if (MSP_SUCCESS != ret) {
    printf("离线语法识别出错: %d \n", ret);
  }
}

int main(int argc, char **argv) {
  // ros初始化
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("speech_recognition_node");
  auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", 1000);
  auto sub_awake = node->create_subscription<std_msgs::msg::Int32>("/mic/awake/angle", 10, awake_callback);
  rclcpp::Rate loop_rate(10);
  RCLCPP_INFO(node->get_logger(), "%s\n", "init success");

  //关键词识别登录初始化
  const char *login_config = "appid = 93f704a1";  //登录参数
  int ret = 0;
  ret = MSPLogin(NULL, NULL, login_config);
  if (MSP_SUCCESS != ret) {
    printf("登录失败：%d\n", ret);
  }

  while (rclcpp::ok()) {
    if (Microsoft_CognitiveServices) {
      //本地识别完成
      std::string xml(g_result);
      // 使用正则表达式提取 confidence
      std::regex confidence_regex("<confidence>(\\d+)</confidence>");
      std::smatch confidence_match;
      if (std::regex_search(xml, confidence_match, confidence_regex)) {
        int confidence = std::stoi(confidence_match[1]);
        if (confidence >= 40) {
          // 使用正则表达式提取 rawtext
          std::regex rawtext_regex("<rawtext>([^<]+)</rawtext>");
          std::smatch rawtext_match;
          if (std::regex_search(xml, rawtext_match, rawtext_regex)) {
            std::string rawtext = rawtext_match[1];
            // 输出提取的数据
            std::cout << "Confidence: " << confidence << std::endl;
            std::cout << "rawtext: " << rawtext << std::endl;
            if (rawtext == "前进一步") {
              std::cout << "前进一步" << std::endl;
            } else if (rawtext == "后退一步") {
              std::cout << "后退一步" << std::endl;
            } else if (rawtext == "转1圈") {
              std::cout << "转1圈" << std::endl;
            }
          }
        } 

      } 
      processSpeechRecognizer();
      Microsoft_CognitiveServices = 0;
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
