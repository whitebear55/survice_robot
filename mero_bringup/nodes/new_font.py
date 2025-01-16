import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QFontDatabase

if __name__ == '__main__':
    app = QApplication(sys.argv)

    # 글꼴 파일 경로 설정
    font_path = 'image/BMHANNAPro.ttf'  # 새로운 글꼴 파일 경로로 수정

    # 글꼴 등록
    font_id = QFontDatabase.addApplicationFont(font_path)
    if font_id != -1:
        font_families = QFontDatabase.applicationFontFamilies(font_id)
        if len(font_families) > 0:
            font_family = font_families[0]
            print(f"새로운 글꼴 '{font_family}' 등록 완료")
        else:
            print("글꼴 파일을 등록할 수 없습니다.")
    else:
        print("글꼴 파일을 등록할 수 없습니다.")

    sys.exit(app.exec())
