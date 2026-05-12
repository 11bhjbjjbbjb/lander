#!/usr/bin/env python3
# encoding: utf-8

from pathlib import Path

import dashscope  # type: ignore

# ---------------------------------------------------------------------------
# 国内阿里云配置：直接硬编码，简化实验部署
# ---------------------------------------------------------------------------
api_key = 'sk-1e51fca58d794323a0bc188bcd2cb2eb'
base_url = 'https://dashscope.aliyuncs.com/compatible-mode/v1'
dashscope.api_key = api_key

# ---------------------------------------------------------------------------
# 默认模型/资源参数，供节点直接引用
# ---------------------------------------------------------------------------
default_llm_model = 'qwen-max-latest'
default_vllm_model = 'qwen-vl-max-latest'
default_tts_model = 'sambert-zhinan-v1'
default_asr_model = 'paraformer-realtime-v2'
default_voice_model = ''
ASR_LANGUAGE = 'Chinese'

_code_path = Path(__file__).resolve().parent
_audio_path = _code_path / 'resources' / 'audio'
_audio_path_en = _audio_path / 'en'


def _audio_base() -> Path:
    return _audio_path if ASR_LANGUAGE.lower() == 'chinese' else _audio_path_en  


def get_audio_path(filename: str) -> str:
    return str(_audio_base() / filename)


recording_audio_path = get_audio_path('recording.wav')
tts_audio_path = get_audio_path('tts_audio.wav')
start_audio_path = get_audio_path('start_audio.wav')
wakeup_audio_path = get_audio_path('wakeup.wav')
error_audio_path = get_audio_path('error.wav')
no_voice_audio_path = get_audio_path('no_voice.wav')
dong_audio_path = get_audio_path('dong.wav')
record_finish_audio_path = get_audio_path('record_finish.wav')
start_track_audio_path = get_audio_path('start_track.wav')
track_fail_audio_path = get_audio_path('track_fail.wav')
