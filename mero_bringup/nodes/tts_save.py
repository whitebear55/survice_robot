from gtts import gTTS

def save_text_to_speech(text, output_file):
    tts = gTTS(text=text, lang="ko")  # TTS 엔진과 언어 설정
    tts.save(output_file)  # MP3 파일로 저장

# 텍스트를 MP3 파일로 저장
text = "안녕하세요, 저는 메로입니다. 학과 방문을 환영합니다!"  # 저장할 텍스트
output_file = "audio/intro.mp3"  # 저장할 MP3 파일 경로
save_text_to_speech(text, output_file)
