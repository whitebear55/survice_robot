import rospy, subprocess, sys, time
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QHBoxLayout, QVBoxLayout, QWidget, QStackedWidget, QMessageBox
from PyQt5.QtGui import QPixmap, QFont, QMovie
from PyQt5.QtCore import Qt, QTimer, QSize
from geometry_msgs.msg import Twist        
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty
from playsound import playsound
from std_msgs.msg import String
from std_msgs.msg import Int32

# font-weight: bold;  /* 글꼴 두껍게 설정 */
TEST_FLAG = False    # move_base/cancels(actionlib_msgs/GoalID), move_base/clear_costmaps

LAB_BUTTON_STYLE_1 ="""
        QPushButton {
            min-width: 177px; 
            min-height: 67px;
            border: 2px solid black;  /* 테두리 색상 및 두께 설정 */
            background-color: rgb(30, 180, 210);  /* 진한 하늘색 배경색 설정 */;
            color: white;
            padding: 10px 20px;
            border-radius: 15px;
            text-align: left;  /* 텍스트 왼쪽 정렬 */
        }
        QPushButton:hover {
            background-color: rgb(70, 140, 180);
        }
        QPushButton:pressed {
            background-color: rgb(0, 100, 140);
            padding-top: 12px;
            padding-bottom: 8px;
        }
    """
LAB_BUTTON_STYLE_2 ="""
        QPushButton {
            min-width: 140px; 
            min-height: 85px;
            border: 2px solid black;  /* 테두리 색상 및 두께 설정 */
            background-color: rgb(30, 180, 210);  /* 진한 하늘색 배경색 설정 */;
            color: white;
            padding: 10px 20px;
            border-radius: 15px;
            text-align: left;  /* 텍스트 왼쪽 정렬 */
        }
        QPushButton:hover {
            background-color: rgb(70, 140, 180);
        }
        QPushButton:pressed {
            background-color: rgb(0, 100, 140);
            padding-top: 12px;
            padding-bottom: 8px;
        }
    """
OFFICE_BUTTON_STYLE = """
        QPushButton {
            min-width: 106px; 
            min-height: 85px;
            border: 2px solid black;  /* 테두리 색상 및 두께 설정 */
            background-color: rgb(255, 145, 100);  
            color: white;
            padding: 10px 20px;
            border-radius: 15px;
            text-align: left;  /* 텍스트 왼쪽 정렬 */
        }
        QPushButton:hover {
            background-color: rgb(225, 105, 60);
        }
        QPushButton:pressed {
            background-color: rgb(185, 65, 20);
            padding-top: 12px;
            padding-bottom: 8px;
        }
    """
PROFESSOR_BUTTON_STYLE_1 = """
        QPushButton {
            min-width: 145px; 
            min-height: 20px;
            border: 2px solid black;  /* 테두리 색상 및 두께 설정 */
            background-color: rgb(170, 205, 100);  
            color: white;
            padding: 10px 20px;
            border-radius: 15px;
            text-align: left;  /* 텍스트 왼쪽 정렬 */
        }
        QPushButton:hover {
            background-color: rgb(130, 165, 60);
        }
        QPushButton:pressed {
            background-color: rgb(100, 135, 30);
            padding-top: 12px;
            padding-bottom: 8px;
        }
    """
MOTION_BUTTON_STYLE = """
        QPushButton {
            min-width: 145px; 
            min-height: 20px;
            border: 3px solid black;  /* 테두리 색상 및 두께 설정 */
            background-color: rgb(245, 245, 150);  
            color: deep grey;
            padding: 10px 20px;
            border-radius: 15px;
            text-align: center;  /* 텍스트 왼쪽 정렬 */
        }
        QPushButton:hover {
            background-color: rgb(205, 205, 110);
        }
        QPushButton:pressed {
            background-color: rgb(165, 165, 70);
            padding-top: 12px;
            padding-bottom: 8px;
        }
    """

INIT_BUTTON_STYLE = """
        QPushButton {
            min-width: 145px; 
            min-height: 60px;
            border: 4px solid black;  /* 테두리 색상 및 두께 설정 */
            background-color: rgb(255, 255, 255); 
            color: deep grey;
            padding: 10px 20px;
            border-radius: 15px;
            text-align: center;  /* 텍스트 왼쪽 정렬 */
        }
        QPushButton:hover {
            background-color: rgb(225, 225, 225);
        }
        QPushButton:pressed {
            background-color: rgb(185, 185, 185);
            padding-top: 12px;
            padding-bottom: 8px;
        }
    """
RECORD_BUTTON_STYLE = """
        QPushButton {
            min-width: 145px; 
            min-height: 60px;
            border: 4px solid black;  /* 테두리 색상 및 두께 설정 */
            background-color: rgb(255, 130, 130); 
            color: deep grey;
            padding: 10px 20px;
            border-radius: 15px;
            text-align: center;  /* 텍스트 왼쪽 정렬 */
        }
        QPushButton:hover {
            background-color: rgb(225, 100, 100);
        }
        QPushButton:pressed {
            background-color: rgb(185, 60, 60);
            padding-top: 12px;
            padding-bottom: 8px;
        }
    """

PLAY_BUTTON_STYLE = """
        QPushButton {
            min-width: 145px; 
            min-height: 60px;
            border: 4px solid black;  /* 테두리 색상 및 두께 설정 */
            background-color: rgb(130, 185, 130); 
            color: deep grey;
            padding: 10px 20px;
            border-radius: 15px;
            text-align: center;  /* 텍스트 왼쪽 정렬 */
        }
        QPushButton:hover {
            background-color: rgb(100, 155, 100);
        }
        QPushButton:pressed {
            background-color: rgb(60, 115, 60);
            padding-top: 12px;
            padding-bottom: 8px;
        }
    """

BACK_BUTTON_STYLE = """
        QPushButton {
            min-width: 80px; 
            min-height: 20px;
            border: 2px solid black;  /* 테두리 색상 및 두께 설정 */
            background-color: rgb(255, 255, 255); 
            color: deep grey;
            padding: 10px 20px;
            border-radius: 15px;
            text-align: center;  /* 텍스트 왼쪽 정렬 */
        }
        QPushButton:hover {
            background-color: rgb(225, 225, 225);
        }
        QPushButton:pressed {
            background-color: rgb(185, 185, 185);
            padding-top: 12px;
            padding-bottom: 8px;
        }
    """

HOME_BUTTON_STYLE = """
        QPushButton {
            min-width: 15px; 
            min-height: 28px;
            border: 2px solid black;  /* 테두리 색상 및 두께 설정 */
            background-color: rgb(255, 255, 255); 
            color: deep grey;
            padding: 10px 20px;
            border-radius: 15px;
            text-align: center;  /* 텍스트 왼쪽 정렬 */
        }
        QPushButton:hover {
            background-color: rgb(225, 225, 225);
        }
        QPushButton:pressed {
            background-color: rgb(185, 185, 185);
            padding-top: 12px;
            padding-bottom: 8px;
        }
    """

old_gx = 0.0
old_gy = 0.0
old_gz = 0.0
old_oz = 0.0
old_ow = 0.0
old_stamp = rospy.Time()

def status_callback(msg):
    global status
    status = msg

def person_callback(msg):
    global person_flag
    if msg.data == "no":
        person_flag = True
    elif msg.data == "ok":
        person_flag = False

def motion_record_popup():
    msg_box = QMessageBox()
    msg_box.setWindowTitle("모션 저장중")
    msg_box.setText("모션은 10초간 기록됩니다.")
    spacer = QLabel()
    spacer.setMaximumHeight(1)

    # 팝업창 위치 조절
    # msg_box.resize(400, 200)
    msg_box.move(650, 250)
    movie = QMovie("{}/image/loading.gif".format(image_path))
    movie.setScaledSize(QSize(200, 200))  # 원하는 크기로 설정
    label = QLabel(msg_box)
    label.setMovie(movie)
    movie.start()
    msg_box.layout().addWidget(spacer)
    msg_box.layout().addWidget(label)
    timer = QTimer()
    timer.setSingleShot(True)
    timer.timeout.connect(msg_box.close)
    timer.start(30000)
    msg_box.exec_() 

def on_record1_click():
    global test_flag
    audio_file = "{}/audio/newdeliverymotionq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    audio_file = "{}/audio/record_10sec.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(motion_widget, "확인창", "새로운 배달 모션을 기록할까요?\n(모션은 10초간 기록됩니다.)", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/newdeliverymotiongo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        # time.sleep(3)
        if test_flag == False:
            record1 = Int32()
            record1.data = 1
            record_pub.publish(record1)
            motion_record_popup()
            audio_file = "{}/audio/record_finish.mp3".format(image_path)  # 재생할 오디오 파일 경로
            playsound(audio_file)
        print("on_record1_click")
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_record2_click():
    global test_flag
    audio_file = "{}/audio/newguidemotionq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    audio_file = "{}/audio/record_10sec.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(motion_widget, "확인창", "새로운 안내 모션을 기록할까요?\n(모션은 10초간 기록됩니다.)", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/newguidemotiongo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        # time.sleep(3)
        if test_flag == False:
            record2 = Int32()
            record2.data = 2
            record_pub.publish(record2)
            motion_record_popup()
            audio_file = "{}/audio/record_finish.mp3".format(image_path)  # 재생할 오디오 파일 경로
            playsound(audio_file)
        print("on_record2_click")
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_record3_click():
    global test_flag
    audio_file = "{}/audio/newtestmotionq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    audio_file = "{}/audio/record_10sec.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(motion_widget, "확인창", "테스트 모션을 기록할까요?\n(모션은 10초간 기록됩니다.)", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/newtestmotiongo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        # time.sleep(3)
        if test_flag == False:
            record3 = Int32()
            record3.data = 3
            record_pub.publish(record3)
            motion_record_popup()
            audio_file = "{}/audio/record_finish.mp3".format(image_path)  # 재생할 오디오 파일 경로
            playsound(audio_file)
        print("on_record3_click")
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_play1_click():
    global test_flag
    audio_file = "{}/audio/savedeliverymotionq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(motion_widget, "확인창", "저장된 배달 모션을 재생할까요?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/savedeliverymotiongo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        # time.sleep(3)
        if test_flag == False:
            play1 = Int32()
            play1.data = 1
            play_pub.publish(play1)
        print("on_play1_click")
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_play2_click():
    global test_flag
    audio_file = "{}/audio/saveguidemotionq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(motion_widget, "확인창", "저장된 안내 모션을 재생할까요?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/saveguidemotiongo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        # time.sleep(3)
        if test_flag == False:
            play2 = Int32()
            play2.data = 2
            play_pub.publish(play2)
        print("on_play2_click")
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_play3_click():
    global test_flag
    audio_file = "{}/audio/savetestmotionq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(motion_widget, "확인창", "테스트 모션을 재생할까요?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/savetestmotiongo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        # time.sleep(3)
        if test_flag == False:
            play3 = Int32()
            play3.data = 3
            play_pub.publish(play3)
        print("on_play3_click")
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_homepos_click():
    global test_flag
    audio_file = "{}/audio/robotarmhomeposq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(motion_widget, "확인창", "로봇팔을 초기위치로 복귀할까요?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/robotarmhomeposgo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        # time.sleep(3)
        if test_flag == False:
            play10 = Int32()
            play10.data = 10
            play_pub.publish(play10)
        print("on_homepos_click")
    else:
        print("아니오 버튼이 클릭되었습니다.")

def cancel_callback(msg):
    global old_stamp
    if msg.stamp.secs != old_stamp:
        old_stamp = msg.stamp.secs
        print("경로 안내중입니다. 비켜주세요")
        audio_file = "{}/audio/person_detect.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        time.sleep(3)

def keep_going_callback(msg):
    global old_gx, old_gy, old_gz, old_oz, old_ow
    if msg.data == "ok":
        print("감사합니다")
        audio_file = "{}/audio/thank_you.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        command = "rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \"header:\n  seq: 0\n  stamp:\n    secs: {}\n    nsecs: {}\n  frame_id: 'map'\npose:\n  position:\n    x: {}\n    y: {}\n    z: {}\n  orientation:\n    x: 0.0\n    y: 0.0\n    z: {}\n    w: {}\"".format(
        rospy.Time.now().secs, rospy.Time.now().nsecs, old_gx, old_gy, old_gz, old_oz, old_ow)
        time.sleep(3)  # 실행할 명령어를 전송하기 위해 잠시 대기
        process = subprocess.Popen(command, shell=True)
        process.wait()

def clear_costmaps():
    # clear_costmaps 서비스 호출
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps_service()
        rospy.loginfo("Costmaps cleared")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call clear_costmaps service: %s", e)

def on_button_chacha_click():
    audio_file = "{}/audio/mero_intro.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    audio_file = "{}/audio/mero_intro2.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)

def on_button_back_click():
    stacked_widget.setCurrentIndex(0)  # 다시 초기 화면으로 이동

def on_button_map_click():
    global test_flag
    audio_file = "{}/audio/guide_page.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    stacked_widget.setCurrentIndex(1)  # 다른 화면으로 이동
    if test_flag == False:
        guide_popup()

def on_button_arm_click():
    audio_file = "{}/audio/motion_page.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    stacked_widget.setCurrentIndex(2)  # 다른 화면으로 이동

def goal_change(gx,gy,gz,oz,ow):
    global old_gx, old_gy, old_gz, old_oz, old_ow
    global test_flag, status, person_flag
    # 명령어 실행
    if test_flag == False:
        clear_costmaps()    

    command = "rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \"header:\n  seq: 0\n  stamp:\n    secs: {}\n    nsecs: {}\n  frame_id: 'map'\npose:\n  position:\n    x: {}\n    y: {}\n    z: {}\n  orientation:\n    x: 0.0\n    y: 0.0\n    z: {}\n    w: {}\"".format(
        rospy.Time.now().secs, rospy.Time.now().nsecs, gx, gy, gz, oz, ow)
    # rospy.loginfo("Executing command: %s", command)
    time.sleep(1)  # 실행할 명령어를 전송하기 위해 잠시 대기

    # 명령어 실행 및 종료 대기
    if test_flag == False:
        process = subprocess.Popen(command, shell=True)
        old_gx = gx
        old_gy = gy
        old_gz = gz
        old_oz = oz
        old_ow = ow
        person_flag = False
        retry_flag = False
        process.wait()
        msg_box = QMessageBox()
        audio_file = "{}/audio/guide_start.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)

        while (status.data != "goal_reached"):
            if (person_flag):
                if retry_flag == False:
                    audio_file = "{}/audio/person_detect.mp3".format(image_path)  # 재생할 오디오 파일 경로
                    playsound(audio_file)
                    retry_flag = True
                else:
                    pass
            else:
                if retry_flag:
                    print("감사합니다")
                    audio_file = "{}/audio/thank_you.mp3".format(image_path)  # 재생할 오디오 파일 경로
                    playsound(audio_file)
                    command = "rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \"header:\n  seq: 0\n  stamp:\n    secs: {}\n    nsecs: {}\n  frame_id: 'map'\npose:\n  position:\n    x: {}\n    y: {}\n    z: {}\n  orientation:\n    x: 0.0\n    y: 0.0\n    z: {}\n    w: {}\"".format(rospy.Time.now().secs, rospy.Time.now().nsecs, old_gx, old_gy, old_gz, old_oz, old_ow)
                    time.sleep(1)  # 실행할 명령어를 전송하기 위해 잠시 대기
                    process = subprocess.Popen(command, shell=True)  
                retry_flag = False  
        print("Finish")
        # msg_box.close()
        cmd2 = String()
        cmd2.data = "uturn"
        cmd_pub.publish(cmd2)
        time.sleep(10)  # 실행할 명령어를 전송하기 위해 잠시 대기
        play3 = Int32()
        play3.data = 3
        play_pub.publish(play3)
        time.sleep(1)  # 실행할 명령어를 전송하기 위해 잠시 대기
        play2 = Int32()
        play2.data = 2
        play_pub.publish(play2)
        time.sleep(1)  # 실행할 명령어를 전송하기 위해 잠시 대기
        audio_file = "{}/audio/back_home_q.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        msg_box = QMessageBox.question(map_widget, "확인창", "목적지에 도착하였습니다. 복귀할까요?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if msg_box == QMessageBox.Yes:
            old_gx = 16.913
            old_gy = -37.626
            old_gz = 0.0
            old_oz = 0.7
            old_ow = 0.7
            person_flag = False
            retry_flag = False
            audio_file = "{}/audio/back_home_go.mp3".format(image_path)  # 재생할 오디오 파일 경로
            playsound(audio_file)
            clear_costmaps() 
            status.data = "go"
            time.sleep(1)  # 실행할 명령어를 전송하기 위해 잠시 대기
            command2 = "rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \"header:\n  seq: 0\n  stamp:\n    secs: {}\n    nsecs: {}\n  frame_id: 'map'\npose:\n  position:\n    x: {}\n    y: {}\n    z: {}\n  orientation:\n    x: 0.0\n    y: 0.0\n    z: {}\n    w: {}\"".format(rospy.Time.now().secs, rospy.Time.now().nsecs, 16.913, -36.626, 0.0, 0.7, 0.7)
            time.sleep(1)  # 실행할 명령어를 전송하기 위해 잠시 대기
            process = subprocess.Popen(command2, shell=True)
            process.wait()  
            # time.sleep(5)  # 실행할 명령어를 전송하기 위해 잠시 대기
            while (status.data != "goal_reached"):
                if (person_flag):
                    if retry_flag == False:
                        audio_file = "{}/audio/person_detect.mp3".format(image_path)  # 재생할 오디오 파일 경로
                        playsound(audio_file)
                        retry_flag = True
                    else:
                        pass
                else:
                    if retry_flag:
                        print("감사합니다")
                        audio_file = "{}/audio/thank_you.mp3".format(image_path)  # 재생할 오디오 파일 경로
                        playsound(audio_file)
                        command = "rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \"header:\n  seq: 0\n  stamp:\n    secs: {}\n    nsecs: {}\n  frame_id: 'map'\npose:\n  position:\n    x: {}\n    y: {}\n    z: {}\n  orientation:\n    x: 0.0\n    y: 0.0\n    z: {}\n    w: {}\"".format(rospy.Time.now().secs, rospy.Time.now().nsecs, 16.913, -36.626, 0.0, 0.7, 0.7)
                        time.sleep(1)  # 실행할 명령어를 전송하기 위해 잠시 대기
                        process = subprocess.Popen(command, shell=True)  
                    retry_flag = False  
                     
            time.sleep(5)  # 실행할 명령어를 전송하기 위해 잠시 대기
            cmd3 = String()
            cmd3.data = "uturn"
            cmd_pub.publish(cmd3) 
            time.sleep(20)  # 실행할 명령어를 전송하기 위해 잠시 대기
            audio_file = "{}/audio/guide_finish.mp3".format(image_path)  # 재생할 오디오 파일 경로
            playsound(audio_file)
            status.data = "go"

            order2 = String()
            order2.data = "navi off" #"roslaunch mero_bringup mero_navigation.launch"   
            order_pub.publish(order2)  
            time.sleep(1)  # 실행할 명령어를 전송하기 위해 잠시 대기

        else:
            print("아니오 버튼이 클릭되었습니다.")

def on_508_click():
    # print("기전시스템실험실로 안내하겠습니다.")
    # 목적지 좌표 설정
    goal_position_x = 53.132
    goal_position_y = -40.303
    goal_position_z = 0.0
    goal_orientation_z = 0.0
    goal_orientation_w = 1.0
    goal_title = "기전시스템실험실"
    
    audio_file = "{}/audio/508q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/508go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")
 
def on_509_click():
    # print("지능생산및자동화실험실로 안내하겠습니다.")
    # 목적지 좌표 설정
    goal_position_x = 35.612
    goal_position_y = -40.24
    goal_position_z = 0.0
    goal_orientation_z = 0.0
    goal_orientation_w = 1.0
    goal_title = "지능생산및자동화실험실"

    audio_file = "{}/audio/509q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/509go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")
    
def on_510_click():
    # print("지능시스템및감성공학실험실로 안내하겠습니다.")
    # 목적지 좌표 설정
    goal_position_x = 24.54
    goal_position_y = -40.327
    goal_position_z = 0.0
    goal_orientation_z = 0.0
    goal_orientation_w = 1.0
    goal_title = "지능시스템및감성공학실험실"

    audio_file = "{}/audio/510q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/510go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_512_click():
    # print("로보틱스실험실로 안내하겠습니다.")
    # 목적지 좌표 설정
    goal_position_x = 18.972
    goal_position_y = -40.378
    goal_position_z = 0.0
    goal_orientation_z = 0.0
    goal_orientation_w = 1.0
    goal_title = "로보틱스실험실"

    audio_file = "{}/audio/512q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/512go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_514_click():
    # print("영상시스템실험실로 안내하겠습니다.")
    # 목적지 좌표 설정
    goal_position_x = 7.9983
    goal_position_y = -40.2
    goal_position_z = 0.0
    goal_orientation_z = 1.0
    goal_orientation_w = 0.0
    goal_title = "영상시스템실험실"

    audio_file = "{}/audio/514q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/514go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_515_click():
    # print("기계역학실험실로 안내하겠습니다.")
    # 목적지 좌표 설정
    goal_position_x = 37.418
    goal_position_y = -40.166
    goal_position_z = 0.0
    goal_orientation_z = 0.0
    goal_orientation_w = 1.0
    goal_title = "기계역학실험실"

    audio_file = "{}/audio/515q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/515go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")
    
def on_517_click():
    # print("학과사무실로 안내하겠습니다.")
    goal_position_x = 21.119
    goal_position_y = -40.276
    goal_position_z = 0.0
    goal_orientation_z = 0.0
    goal_orientation_w = 1.0
    goal_title = "학과사무실"
 
    audio_file = "{}/audio/517q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/517go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")
    
def on_519_click():
    # print("생체공학실험실로 안내하겠습니다.")
    goal_position_x = -2.7676
    goal_position_y = -28.923
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "생체공학실험실"
    
    audio_file = "{}/audio/519q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/519go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_520_click():
    # print("전산설계자동화실험실로 안내하겠습니다.")
    # 목적지 좌표 설정
    goal_position_x = -2.7255
    goal_position_y = -23.271
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "전산설계자동화실험실"

    audio_file = "{}/audio/520q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/520go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_521_click():
    # print("지능차량시스템실험실로 안내하겠습니다.")
    goal_position_x = -2.9438
    goal_position_y = -5.6114
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "지능차량시스템실험실"

    audio_file = "{}/audio/521q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/521go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_522_click():
    # print("안미치코 교수연구실로 안내하겠습니다.")
    goal_position_x = -2.7944
    goal_position_y = -35.763
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "안미치코 교수연구실"
    
    audio_file = "{}/audio/522q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/522go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_523_click():
    # print("정슬 교수연구실로 안내하겠습니다.")
    goal_position_x = -2.8874
    goal_position_y = -32.802
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "정슬 교수연구실"

    audio_file = "{}/audio/523q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/523go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_524_click():
    # print("노명규 교수연구실로 안내하겠습니다.")
    goal_position_x = -2.8485
    goal_position_y = -28.589
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "노명규 교수연구실"
    
    audio_file = "{}/audio/524q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/524go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_525_click():
    # print("박영우 교수연구실로 안내하겠습니다.")
    goal_position_x = -2.7431
    goal_position_y = -25.335
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "박영우 교수연구실"
    
    audio_file = "{}/audio/525q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/525go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")
 
def on_526_click():
    # print("이지홍 교수연구실로 안내하겠습니다.")
    goal_position_x = -2.8257
    goal_position_y = -21.459
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "이지홍 교수연구실"
    
    audio_file = "{}/audio/526q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/526go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")
 
def on_527_click():
    # print("고윤호 교수연구실로 안내하겠습니다.")
    goal_position_x = -2.8661
    goal_position_y = -18.315
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "고윤호 교수연구실"
    
    audio_file = "{}/audio/527q.mp3".format(image_path)  # 재생할 오디오 파일 경로msg_box
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/527go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")
 
def on_528_click():
    # print("김성수 교수연구실로 안내하겠습니다.")
    goal_position_x = -2.8324
    goal_position_y = -14.199
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "김성수 교수연구실"
    
    audio_file = "{}/audio/528q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/528go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_529_click():
    # print("원문철 교수연구실로 안내하겠습니다.")
    goal_position_x = -2.9073
    goal_position_y = -10.931
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "원문철 교수연구실"
    
    audio_file = "{}/audio/529q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/529go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_530_click():
    # print("양석조 교수연구실로 안내하겠습니다.")
    goal_position_x = -2.9791
    goal_position_y = -6.9965
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "양석조 교수연구실"
    
    audio_file = "{}/audio/530q.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/530go.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_toilet_click():
    # print("화장실로 안내하겠습니다.")
    goal_position_x = 3.0365
    goal_position_y = -40.064
    goal_position_z = 0.0
    goal_orientation_z = 1.0
    goal_orientation_w = 0.0
    goal_title = "화장실"
 
    audio_file = "{}/audio/toiletq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/toiletgo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_elevator_main_click():
    # print("본관 엘리베이터로 안내하겠습니다.")
    goal_position_x = 16.913
    goal_position_y = -36.626
    goal_position_z = 0.0
    goal_orientation_z = 0.7
    goal_orientation_w = 0.7
    goal_title = "본관 엘리베이터"
 
    audio_file = "{}/audio/mainelevq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/mainelevgo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_elevator_extra_click():
    # print("별관 엘리베이터로 안내하겠습니다.")
    goal_position_x = -1.0
    goal_position_y = 0.0
    goal_position_z = 0.0
    goal_orientation_z = 0.0
    goal_orientation_w = 1.0
    goal_title = "별관 엘리베이터"
 
    audio_file = "{}/audio/extraelevq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "{}로 안내를 진행할까요?".format(goal_title), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/extraelevgo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        goal_change(goal_position_x, goal_position_y, goal_position_z, goal_orientation_z, goal_orientation_w)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_coffee_click():
    # print("별관 엘리베이터로 안내하겠습니다.")
    audio_file = "{}/audio/coffeeq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "음료배달 모드로 전환할까요?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/coffeego.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
    else:
        print("아니오 버튼이 클릭되었습니다.")


def on_heart_click():
    # print("하트 동작을 보여드릴까요?")
    audio_file = "{}/audio/heartq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "하트 동작을 보여드릴까요?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/heartgo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_dance_click():
    # print("댄스 동작을 보여드릴까요?")
    audio_file = "{}/audio/danceq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "댄스 동작을 보여드릴까요?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/dancego.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_user_click():
    # print("실시간 교시 모드로 전환할까요?")
    audio_file = "{}/audio/rtmotionq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "실시간 교시 모드로 전환할까요?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/rtmotiongo.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        # rtflag = Int32()
        # rtflag.data = 1
        # rtflag_pub.publish(rtflag)
        # time.sleep(1)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def on_save_click():
    # print("모션 저장 모드로 전환할까요?")
    audio_file = "{}/audio/motionsaveq.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox.question(map_widget, "확인창", "모션 저장 모드로 전환할까요?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
    if msg_box == QMessageBox.Yes:
        audio_file = "{}/audio/motionsavego.mp3".format(image_path)  # 재생할 오디오 파일 경로
        playsound(audio_file)
        stacked_widget.setCurrentIndex(3)  # 다른 화면으로 이동
        # rtflag = Int32()
        # rtflag.data = 0
        # rtflag_pub.publish(rtflag)
        # time.sleep(1)
    else:
        print("아니오 버튼이 클릭되었습니다.")

def test_toggle_button_clicked():
    global test_flag
    if test_toggle_button.isChecked():
        test_toggle_label.setText("테스트 모드: ON")
        test_flag = True
    else:
        test_toggle_label.setText("테스트 모드: OFF")
        test_flag = False

def guide_popup():
    global test_flag
    audio_file = "{}/audio/guide_mode.mp3".format(image_path)  # 재생할 오디오 파일 경로
    playsound(audio_file)
    msg_box = QMessageBox()
    msg_box.setWindowTitle("안내 모드 전환중")
    msg_box.setText("저에게서 떨어져 주세요")
    spacer = QLabel()
    spacer.setMaximumHeight(1)

    # 팝업창 위치 조절
    # msg_box.resize(400, 200)
    msg_box.move(650, 250)
    movie = QMovie("{}/image/loading.gif".format(image_path))
    movie.setScaledSize(QSize(200, 200))  # 원하는 크기로 설정
    label = QLabel(msg_box)
    label.setMovie(movie)
    movie.start()
    msg_box.layout().addWidget(spacer)
    msg_box.layout().addWidget(label)
    
    # 타이머 생성 및 연결
    timer = QTimer()
    timer.setSingleShot(True)
    timer.timeout.connect(msg_box.close)
    order1 = String()
    order1.data = "navi on" #"roslaunch mero_bringup mero_navigation.launch"
    order_pub.publish(order1)

    print("초기 위치를 설정하겠습니다.")
    command2 = "rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \"{ \
    header: { \
        stamp: {secs: 0, nsecs: 0}, \
        frame_id: 'map' \
    }, \
    pose: { \
        pose: { \
            position: {x: 16.913, y: -36.626, z: 0.0}, \
            orientation: {x: 0.0, y: 0.0, z: -0.7, w: 0.7} \
        }, \
        covariance: [0.25, 0, 0, 0, 0, 0, \
                     0, 0.25, 0, 0, 0, 0, \
                     0, 0, 0, 0, 0, 0, \
                     0, 0, 0, 0, 0, 0, \
                     0, 0, 0, 0, 0, 0, \
                     0, 0, 0, 0, 0, 0.06853891945200942] \
    } \
}\""
    time.sleep(1)  # 실행할 명령어를 전송하기 위해 잠시 대기

    # 명령어 실행 및 종료 대기
    if test_flag == False:
        # subprocess.Popen(command1, shell=True)
        time.sleep(1)  # 실행할 명령어를 전송하기 위해 잠시 대기
        subprocess.Popen(command2, shell=True)

    timer.start(20000)

    msg_box.exec_() 
    time.sleep(1)
    cmd = String()
    cmd.data = "init"
    cmd_pub.publish(cmd)
    time.sleep(1)

if __name__ == '__main__':
    global test_flag, status
    rospy.init_node('gui_node')
    app = QApplication(sys.argv)
    cmd_pub = rospy.Publisher('cmd', String, queue_size=10)     # publisher를 정의해주고, 토픽명, 메세지 타입, 큐사이즈를 설정한다.
    order_pub = rospy.Publisher("order", String, queue_size=10) 
    record_pub = rospy.Publisher("record", Int32, queue_size=10)
    play_pub = rospy.Publisher("play", Int32, queue_size=10)
    rtflag_pub = rospy.Publisher("rtflag", Int32, queue_size=10)
    status_sub = rospy.Subscriber("status", String, status_callback, queue_size=1)
    person_sub = rospy.Subscriber("person", String, person_callback, queue_size=1)
    test_flag = False
    status = String()

    # 이미지 경로 파라미터 받아오기
    image_path = rospy.get_param("~image_path", "")

    # 창 생성
    window = QWidget()
    # 스택 위젯 생성
    stacked_widget = QStackedWidget()

    # 초기 화면
    initial_widget = QWidget()
    initial_widget.setStyleSheet("background-color: white;")  # 배경색을 흰색으로 설정
    initial_layout = QVBoxLayout()
    initial_label = QLabel()
    button_layout = QHBoxLayout()
    # 공백 위젯 생성
    spacer = QLabel()
    spacer.setMaximumHeight(150)
    chacha_label = QLabel()
    pixmap = QPixmap('{}/image/차차_환영.png'.format(image_path))
    pixmap = pixmap.scaled(300, 300, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    chacha_label.setPixmap(pixmap)
    chacha_label.setAlignment(Qt.AlignCenter)

    # 버튼 생성
    buttonechacha = QPushButton()
    buttonechacha.setFont(QFont("BM HANNA Pro", 13))  # 폰트 이름과 크기 설정
    # buttonechacha.setStyleSheet("background-color: transparent;")
    buttonechacha.setStyleSheet("QPushButton { background-color: transparent; color: white; border: none; }")
    buttonechacha.clicked.connect(on_button_chacha_click)
    buttonechacha.setParent(chacha_label)
    buttonechacha.move(470, 0)  # 버튼의 위치 조정
    buttonechacha.resize(300,300)

    button_map = QPushButton("지도 안내")
    button_map.setFont(QFont("BM HANNA Pro", 25))  # 폰트 이름과 크기 설정
    button_map.clicked.connect(on_button_map_click)
    button_map.setStyleSheet(INIT_BUTTON_STYLE)

    click_label = QLabel("Click!!")
    click_label.setAlignment(Qt.AlignCenter)  # 텍스트를 중앙에 정렬
    font = QFont("BM HANNA Pro", 30)
    click_label.setStyleSheet("color: rgb(0, 150, 180);")
    click_label.setFont(font)

    button_arm = QPushButton("로봇팔 동작")
    button_arm.setFont(QFont("BM HANNA Pro", 25))  # 폰트 이름과 크기 설정
    button_arm.clicked.connect(on_button_arm_click)
    button_arm.setStyleSheet(INIT_BUTTON_STYLE)

    pixmap = QPixmap('{}/image/department_name.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    pixmap = pixmap.scaled(650, 650, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    initial_label.setPixmap(pixmap)
    initial_label.setAlignment(Qt.AlignCenter)

    text_label = QLabel("학과 방문을 환영합니다!")
    text_label.setAlignment(Qt.AlignCenter)  # 텍스트를 중앙에 정렬
    font = QFont("BM HANNA Pro", 35)
    text_label.setFont(font)

    test_toggle_label = QLabel("테스트 모드: OFF")
    test_toggle_button = QPushButton("모드 전환")
    test_toggle_button.setCheckable(True)  # 토글 버튼으로 설정
    test_toggle_button.clicked.connect(test_toggle_button_clicked)

    test_layout = QHBoxLayout()
    test_layout.addStretch(1)  # 왼쪽 여백 추가
    # test_layout.addWidget(initial_label)
    test_layout.addWidget(test_toggle_label)
    test_layout.addWidget(test_toggle_button)

    # pixmap = pixmap.scaled(1024, 860, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    button_layout.addStretch(1)  # 왼쪽 여백 추가
    button_layout.addWidget(button_map)
    button_layout.addStretch(1)  # 왼쪽 여백 추가
    button_layout.addWidget(click_label)
    button_layout.addStretch(1)  # 왼쪽 여백 추가
    button_layout.addWidget(button_arm)
    button_layout.addStretch(1)  # 오른쪽 여백 추가
    button_layout.addWidget(spacer)
    
    initial_layout.addLayout(test_layout)
    initial_layout.addWidget(initial_label)
    initial_layout.addWidget(text_label)
    initial_layout.addWidget(chacha_label)
    initial_layout.addLayout(button_layout)

    # initial_layout.addWidget(button1)
    initial_widget.setLayout(initial_layout)

    # 이미지 레이블 생성
    map_widget = QWidget()
    map_widget.setStyleSheet("background-color: white;")  # 배경색을 흰색으로 설정
    map_layout = QVBoxLayout()
    map_label = QLabel("5층 평면도")
    pixmap = QPixmap('{}/image/5f_평면도_4.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    pixmap = pixmap.scaled(1240, 1024, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    map_label.setPixmap(pixmap)
    map_layout.addWidget(map_label)
    map_widget.setLayout(map_layout)

    button_back = QPushButton("Back")
    button_back.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_back.setStyleSheet(BACK_BUTTON_STYLE)
    button_back.clicked.connect(on_button_back_click)
    button_back.setParent(map_label)
    button_back.move(1115, 0)  # 버튼의 위치 조정

    button508 = QPushButton("508\n기전시스템실험실")
    button508.setFont(QFont("BM HANNA Pro", 16))  # 폰트 이름과 크기 설정
    button508.setStyleSheet(LAB_BUTTON_STYLE_2)
    button508.clicked.connect(on_508_click)
    button508.setParent(map_label)
    button508.move(20, 12)  # 버튼의 위치 조정

    button509 = QPushButton("509\n지능생산 및\n자동화실험실")
    button509.setFont(QFont("BM HANNA Pro", 16))  # 폰트 이름과 크기 설정
    button509.setStyleSheet(LAB_BUTTON_STYLE_2)
    button509.clicked.connect(on_509_click)
    button509.setParent(map_label)
    button509.move(220, 12)  # 버튼의 위치 조정

    button510 = QPushButton("510\n지능시스템 및\n감성공학실험실")
    button510.setFont(QFont("BM HANNA Pro", 16))  # 폰트 이름과 크기 설정
    button510.setStyleSheet(LAB_BUTTON_STYLE_2)
    button510.clicked.connect(on_510_click)
    button510.setParent(map_label)
    button510.move(420, 12)  # 버튼의 위치 조정

    button512 = QPushButton("512\n로보틱스실험실")
    button512.setFont(QFont("BM HANNA Pro", 16))  # 폰트 이름과 크기 설정
    button512.setStyleSheet(LAB_BUTTON_STYLE_2)
    button512.clicked.connect(on_512_click)
    button512.setParent(map_label)
    button512.move(620, 12)  # 버튼의 위치 조정

    button514 = QPushButton("514\n영상시스템실험실")
    button514.setFont(QFont("BM HANNA Pro", 16))  # 폰트 이름과 크기 설정
    button514.setStyleSheet(LAB_BUTTON_STYLE_2)
    button514.clicked.connect(on_514_click)
    button514.setParent(map_label)
    button514.move(820, 12)  # 버튼의 위치 조정

    button515 = QPushButton("515\n기계역학실험실")
    button515.setFont(QFont("BM HANNA Pro", 16))  # 폰트 이름과 크기 설정
    button515.setStyleSheet(LAB_BUTTON_STYLE_2)
    button515.clicked.connect(on_515_click)
    button515.setParent(map_label)
    button515.move(20, 144)  # 버튼의 위치 조정

    button517 = QPushButton("517\n학과사무실")
    button517.setFont(QFont("BM HANNA Pro", 16))  # 폰트 이름과 크기 설정
    button517.setStyleSheet(OFFICE_BUTTON_STYLE)
    button517.clicked.connect(on_517_click)
    button517.setParent(map_label)
    button517.move(250, 144)  # 버튼의 위치 조정

        # 버튼 생성
    button519 = QPushButton("519\n생체공학실험실")
    button519.setFont(QFont("BM HANNA Pro", 16))  # 폰트 이름과 크기 설정
    button519.setStyleSheet(LAB_BUTTON_STYLE_1)
    button519.clicked.connect(on_519_click)
    button519.setParent(map_label)
    button519.move(750, 260)  # 버튼의 위치 조정

    # 버튼 생성
    button520 = QPushButton("520\n전산설계자동화실험실")
    button520.setFont(QFont("BM HANNA Pro", 16))  # 폰트 이름과 크기 설정
    button520.setStyleSheet(LAB_BUTTON_STYLE_1)
    button520.clicked.connect(on_520_click)
    button520.setParent(map_label)
    button520.move(750, 365)  # 버튼의 위치 조정

    # 버튼 생성
    button521 = QPushButton("521\n지능차량시스템실험실")
    button521.setFont(QFont("BM HANNA Pro", 16))  # 폰트 이름과 크기 설정
    button521.setStyleSheet(LAB_BUTTON_STYLE_1)
    button521.clicked.connect(on_521_click)
    button521.setParent(map_label)
    button521.move(750, 470)  # 버튼의 위치 조정

    # 버튼 생성
    button522 = QPushButton("522\n안미치코 교수연구실")
    button522.setFont(QFont("BM HANNA Pro", 15))  # 폰트 이름과 크기 설정
    button522.setStyleSheet(PROFESSOR_BUTTON_STYLE_1)
    button522.clicked.connect(on_522_click)
    button522.setParent(map_label)
    button522.move(1020, 84)  # 버튼의 위치 조정

    # 버튼 생성
    button523 = QPushButton("523\n정슬 교수연구실")
    button523.setFont(QFont("BM HANNA Pro", 15))  # 폰트 이름과 크기 설정
    button523.setStyleSheet(PROFESSOR_BUTTON_STYLE_1)
    button523.clicked.connect(on_523_click)
    button523.setParent(map_label)
    button523.move(1020, 154)  # 버튼의 위치 조정

    # 버튼 생성
    button524 = QPushButton("524\n노명규 교수연구실")
    button524.setFont(QFont("BM HANNA Pro", 15))  # 폰트 이름과 크기 설정
    button524.setStyleSheet(PROFESSOR_BUTTON_STYLE_1)
    button524.clicked.connect(on_524_click)
    button524.setParent(map_label)
    button524.move(1020, 224)  # 버튼의 위치 조정

    # 버튼 생성
    button525 = QPushButton("525\n박영우 교수연구실")
    button525.setFont(QFont("BM HANNA Pro", 15))  # 폰트 이름과 크기 설정
    button525.setStyleSheet(PROFESSOR_BUTTON_STYLE_1)
    button525.clicked.connect(on_525_click)
    button525.setParent(map_label)
    button525.move(1020, 294)  # 버튼의 위치 조정

    # 버튼 생성
    button526 = QPushButton("526\n이지홍 교수연구실")
    button526.setFont(QFont("BM HANNA Pro", 15))  # 폰트 이름과 크기 설정
    button526.setStyleSheet(PROFESSOR_BUTTON_STYLE_1)
    button526.clicked.connect(on_526_click)
    button526.setParent(map_label)
    button526.move(1020, 364)  # 버튼의 위치 조정

    # 버튼 생성
    button527 = QPushButton("527\n고윤호 교수연구실")
    button527.setFont(QFont("BM HANNA Pro", 15))  # 폰트 이름과 크기 설정
    button527.setStyleSheet(PROFESSOR_BUTTON_STYLE_1)
    button527.clicked.connect(on_527_click)
    button527.setParent(map_label)
    button527.move(1020, 434)  # 버튼의 위치 조정

    # 버튼 생성
    button528 = QPushButton("528\n김성수 교수연구실")
    button528.setFont(QFont("BM HANNA Pro", 15))  # 폰트 이름과 크기 설정
    button528.setStyleSheet(PROFESSOR_BUTTON_STYLE_1)
    button528.clicked.connect(on_528_click)
    button528.setParent(map_label)
    button528.move(1020, 504)  # 버튼의 위치 조정

    # 버튼 생성
    button529 = QPushButton("529\n원문철 교수연구실")
    button529.setFont(QFont("BM HANNA Pro", 15))  # 폰트 이름과 크기 설정
    button529.setStyleSheet(PROFESSOR_BUTTON_STYLE_1)
    button529.clicked.connect(on_529_click)
    button529.setParent(map_label)
    button529.move(1020, 574)  # 버튼의 위치 조정

    # 버튼 생성
    button530 = QPushButton("530\n양석조 교수연구실")
    button530.setFont(QFont("BM HANNA Pro", 15))  # 폰트 이름과 크기 설정
    button530.setStyleSheet(PROFESSOR_BUTTON_STYLE_1)
    button530.clicked.connect(on_530_click)
    button530.setParent(map_label)
    button530.move(1020, 644)  # 버튼의 위치 조정

    # 버튼 생성
    buttontoilet = QPushButton()
    buttontoilet.setFont(QFont("BM HANNA Pro", 13))  # 폰트 이름과 크기 설정
    # buttontoilet.setStyleSheet("background-color: transparent;")
    buttontoilet.setStyleSheet("QPushButton { background-color: transparent; color: white; border: none; }")
    buttontoilet.clicked.connect(on_toilet_click)
    buttontoilet.setParent(map_label)
    buttontoilet.move(735, 142)  # 버튼의 위치 조정
    buttontoilet.resize(100,115)

    # 버튼 생성
    buttonelevatormain = QPushButton()
    buttonelevatormain.setFont(QFont("BM HANNA Pro", 13))  # 폰트 이름과 크기 설정
    # buttonelevatormain.setStyleSheet("background-color: transparent;")
    buttonelevatormain.setStyleSheet("QPushButton { background-color: transparent; color: white; border: none; }")
    buttonelevatormain.clicked.connect(on_elevator_main_click)
    buttonelevatormain.setParent(map_label)
    buttonelevatormain.move(465, 142)  # 버튼의 위치 조정
    buttonelevatormain.resize(100,115)

    # 버튼 생성
    buttonelevatorextra = QPushButton()
    buttonelevatorextra.setFont(QFont("BM HANNA Pro", 13))  # 폰트 이름과 크기 설정
    # buttonelevatorextra.setStyleSheet("background-color: transparent;")
    buttonelevatorextra.setStyleSheet("QPushButton { background-color: transparent; color: white; border: none; }")
    buttonelevatorextra.clicked.connect(on_elevator_extra_click)
    buttonelevatorextra.setParent(map_label)
    buttonelevatorextra.move(860, 572)  # 버튼의 위치 조정
    buttonelevatorextra.resize(100,115)

    button_home = QPushButton("음료\n배달")
    button_home.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_home.setStyleSheet(HOME_BUTTON_STYLE)
    button_home.clicked.connect(on_coffee_click)
    button_home.setParent(map_label)
    # button_home.resize(30,30)
    button_home.move(500, 600)  # 버튼의 위치 조정

    arm_widget = QWidget()
    arm_widget.setStyleSheet("background-color: white;")  # 배경색을 흰색으로 설정 
    arm_layout = QVBoxLayout()
    arm_layout2 = QHBoxLayout()
    arm_layout3 = QHBoxLayout()
    arm_layout4 = QHBoxLayout()

    arm_label1 = QLabel()
    arm_label2= QLabel()
    arm_label3= QLabel()
    arm_label4= QLabel()
    arm_label5 = QLabel()
    arm_label6 = QLabel()
    arm_label7 = QLabel()

    spacer = QLabel()
    # spacer.setMaximumHeight(200)

    pixmap1 = QPixmap('{}/image/heart.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    pixmap1 = pixmap1.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    arm_label1.setPixmap(pixmap1)
    pixmap2 = QPixmap('{}/image/dance.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    pixmap2 = pixmap2.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    arm_label2.setPixmap(pixmap2)
    pixmap3 = QPixmap('{}/image/camera.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    pixmap3 = pixmap3.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    arm_label3.setPixmap(pixmap3)
    pixmap4 = QPixmap('{}/image/save.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    pixmap4 = pixmap4.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    arm_label4.setPixmap(pixmap4)
    pixmap5 = QPixmap('{}/image/department_name.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    pixmap5 = pixmap5.scaled(330, 330, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    arm_label6.setPixmap(pixmap5)
    pixmap6 = QPixmap('{}/image/차차_동작.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    pixmap6 = pixmap6.scaled(550, 550, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    arm_label7.setPixmap(pixmap6)

    button_back = QPushButton("Back")
    button_back.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_back.setStyleSheet(BACK_BUTTON_STYLE)
    button_back.clicked.connect(on_button_back_click)
    button_back.setParent(spacer)
    button_back.move(1115, 0)  # 버튼의 위치 조정

    button_move1 = QPushButton()
    # button_move1.setStyleSheet("background-color: transparent;")
    button_move1.setStyleSheet("QPushButton { background-color: transparent; color: white; border: none; }")
    button_move1.clicked.connect(on_heart_click)
    button_move1.setParent(arm_label1)
    button_move1.move(0, 20)  # 버튼의 위치 조정
    button_move1.resize(300,260)

    button_move2 = QPushButton()
    # button_move2.setStyleSheet("background-color: transparent;")
    button_move2.setStyleSheet("QPushButton { background-color: transparent; color: white; border: none; }")
    button_move2.clicked.connect(on_dance_click)
    button_move2.setParent(arm_label2)
    button_move2.move(0, 20)  # 버튼의 위치 조정
    button_move2.resize(300,260)

    button_move3 = QPushButton()
    # button_move3.setStyleSheet("background-color: transparent;")
    button_move3.setStyleSheet("QPushButton { background-color: transparent; color: white; border: none; }")
    button_move3.clicked.connect(on_user_click)
    button_move3.setParent(arm_label3)
    button_move3.move(0, 20)  # 버튼의 위치 조정
    button_move3.resize(300,260)

    button_move4 = QPushButton()
    # button_move4.setStyleSheet("background-color: transparent;")
    button_move4.setStyleSheet("QPushButton { background-color: transparent; color: white; border: none; }")
    button_move4.clicked.connect(on_save_click)
    button_move4.setParent(arm_label4)
    button_move4.move(0, 20)  # 버튼의 위치 조정
    button_move4.resize(300,260)

    button_move11 = QPushButton("하트 모션")
    button_move11.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_move11.clicked.connect(on_heart_click)
    button_move11.setStyleSheet(MOTION_BUTTON_STYLE)
    button_move11.setParent(arm_label5)
    button_move11.move(75, 20)  # 버튼의 위치 조정

    button_move22 = QPushButton("댄스 모션")
    button_move22.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_move22.clicked.connect(on_dance_click)
    button_move22.setStyleSheet(MOTION_BUTTON_STYLE)
    button_move22.setParent(arm_label5)
    button_move22.move(380, 20)  # 버튼의 위치 조정

    button_move33 = QPushButton("실시간 교시")
    button_move33.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_move33.clicked.connect(on_user_click)
    button_move33.setStyleSheet(MOTION_BUTTON_STYLE)
    button_move33.setParent(arm_label5)
    button_move33.move(675, 20)  # 버튼의 위치 조정

    button_move44 = QPushButton("모션 저장")
    button_move44.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_move44.clicked.connect(on_save_click)
    button_move44.setStyleSheet(MOTION_BUTTON_STYLE)
    button_move44.setParent(arm_label5)
    button_move44.move(975, 20)  # 버튼의 위치 조정

    arm_layout2.addStretch()
    arm_layout2.addWidget(arm_label1)    
    arm_layout2.addStretch()
    arm_layout2.addWidget(arm_label2)   
    arm_layout2.addStretch()
    arm_layout2.addWidget(arm_label3)    
    arm_layout2.addStretch()
    arm_layout2.addWidget(arm_label4)  
    arm_layout2.addStretch()
    arm_layout3.addWidget(arm_label5) 
    arm_layout4.addWidget(arm_label6)
    arm_layout4.addStretch()
    arm_layout4.addWidget(arm_label7)
    arm_layout.addWidget(spacer)
    arm_layout.addLayout(arm_layout2) 
    arm_layout.addLayout(arm_layout3) 
    arm_layout.addLayout(arm_layout4)  
 
    arm_widget.setLayout(arm_layout)

    # 초기 화면
    motion_widget = QWidget()
    motion_widget.setStyleSheet("background-color: white;")  # 배경색을 흰색으로 설정
    motion_layout1= QVBoxLayout()
    motion_layout2= QHBoxLayout()
    motion_layout3= QHBoxLayout()
    motion_layout4= QHBoxLayout()

    sequence1_label = QLabel()
    sequence2_label = QLabel()
    sequence3_label = QLabel()
    sequence4_label = QLabel()
    sequence5_label = QLabel()
    sequence6_label = QLabel()
    sequence7_label = QLabel()


    record1_label = QLabel()
    record2_label = QLabel()
    record3_label = QLabel()
    play1_label = QLabel()
    play2_label = QLabel()
    play3_label = QLabel()
    spacer = QLabel()
    spacer.setMaximumHeight(90)
    spacer2 = QLabel()
    spacer2.setMaximumHeight(30)

    seq1_pixmap = QPixmap('{}/image/record.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    seq1_pixmap = seq1_pixmap.scaled(200, 200, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    sequence1_label.setPixmap(seq1_pixmap)
    seq2_pixmap = QPixmap('{}/image/next.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    seq2_pixmap = seq2_pixmap.scaled(80, 80, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    sequence2_label.setPixmap(seq2_pixmap)
    seq3_pixmap = QPixmap('{}/image/pose1.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    seq3_pixmap = seq3_pixmap.scaled(100, 100, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    sequence3_label.setPixmap(seq3_pixmap)
    seq4_pixmap = QPixmap('{}/image/pose2.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    seq4_pixmap = seq4_pixmap.scaled(100, 100, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    sequence4_label.setPixmap(seq4_pixmap)
    seq5_pixmap = QPixmap('{}/image/pose3.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    seq5_pixmap = seq5_pixmap.scaled(100, 100, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    sequence5_label.setPixmap(seq5_pixmap)
    sequence6_label.setPixmap(seq2_pixmap)
    seq7_pixmap = QPixmap('{}/image/play.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    seq7_pixmap = seq7_pixmap.scaled(100, 100, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    sequence7_label.setPixmap(seq7_pixmap)

    button_record1 = QPushButton("배달 모션 기록")
    button_record1.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_record1.setStyleSheet(RECORD_BUTTON_STYLE)
    button_record1.clicked.connect(on_record1_click)
    button_record1.setParent(record1_label)
    # button_record1.resize(300,260)
    button_record1.move(100, 0)  # 버튼의 위치 조정

    button_record2 = QPushButton("안내 모션 기록")
    button_record2.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_record2.setStyleSheet(RECORD_BUTTON_STYLE)
    button_record2.clicked.connect(on_record2_click)
    button_record2.setParent(record2_label)
    # button_record2.resize(300,260)
    button_record2.move(100, 0)  # 버튼의 위치 조정

    button_record3 = QPushButton("테스트 모션 기록")
    button_record3.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_record3.setStyleSheet(RECORD_BUTTON_STYLE)
    button_record3.clicked.connect(on_record3_click)
    button_record3.setParent(record3_label)
    # button_record3.resize(300,260)
    button_record3.move(100, 0)  # 버튼의 위치 조정

    button_play1 = QPushButton("배달 모션 재생")
    button_play1.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_play1.setStyleSheet(PLAY_BUTTON_STYLE)
    button_play1.clicked.connect(on_play1_click)
    button_play1.setParent(play1_label)
    # button_play1.resize(300,260)
    button_play1.move(100, 0)  # 버튼의 위치 조정

    button_play2 = QPushButton("안내 모션 재생")
    button_play2.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_play2.setStyleSheet(PLAY_BUTTON_STYLE)
    button_play2.clicked.connect(on_play2_click)
    button_play2.setParent(play2_label)
    # button_play2.resize(300,260)
    button_play2.move(100, 0)  # 버튼의 위치 조정

    button_play3 = QPushButton("테스트 모션 재생")
    button_play3.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_play3.setStyleSheet(PLAY_BUTTON_STYLE)
    button_play3.clicked.connect(on_play3_click)
    button_play3.setParent(play3_label)
    # button_play3.resize(300,260)
    button_play3.move(100, 0)  # 버튼의 위치 조정

    button_homepos = QPushButton("초기 위치")
    button_homepos.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_homepos.setStyleSheet(INIT_BUTTON_STYLE)
    button_homepos.clicked.connect(on_homepos_click)
    button_homepos.setParent(spacer)
    # button_homepos.resize(300,260)
    button_homepos.move(515, 0)  # 버튼의 위치 조정

    pixmap5 = QPixmap('{}/image/department_name.png'.format(image_path))  # 이미지 파일 경로를 설정하세요
    pixmap5 = pixmap5.scaled(330, 330, Qt.AspectRatioMode.KeepAspectRatio)  # 이미지 크기 조절
    spacer.setPixmap(pixmap5)

    button_back = QPushButton("Back")
    button_back.setFont(QFont("BM HANNA Pro", 20))  # 폰트 이름과 크기 설정
    button_back.setStyleSheet(BACK_BUTTON_STYLE)
    button_back.clicked.connect(on_button_back_click)
    button_back.setParent(spacer)
    button_back.move(1115, 0)  # 버튼의 위치 조정
    
    motion_layout2.addWidget(record1_label)
    motion_layout2.addWidget(record2_label)
    motion_layout2.addWidget(record3_label)

    motion_layout3.addWidget(play1_label)
    motion_layout3.addWidget(play2_label)
    motion_layout3.addWidget(play3_label)

    motion_layout4.addStretch(1)
    motion_layout4.addWidget(sequence1_label)
    motion_layout4.addWidget(sequence2_label)
    motion_layout4.addWidget(sequence3_label)
    motion_layout4.addWidget(sequence4_label)
    motion_layout4.addWidget(sequence5_label)
    motion_layout4.addWidget(sequence6_label)
    motion_layout4.addWidget(sequence7_label)
    motion_layout4.addStretch(1)
    # motion_layout4.addWidget(spacer)

    motion_layout1.addWidget(spacer)
    motion_layout1.addLayout(motion_layout4)
    motion_layout1.addWidget(spacer2)
    motion_layout1.addLayout(motion_layout2)
    motion_layout1.addLayout(motion_layout3)

    motion_widget.setLayout(motion_layout1)

    # 스택 위젯에 위젯 추가
    stacked_widget.addWidget(initial_widget)
    stacked_widget.addWidget(map_widget)
    stacked_widget.addWidget(arm_widget)
    stacked_widget.addWidget(motion_widget)

    # 초기 화면을 현재 위젯으로 설정
    stacked_widget.setCurrentIndex(0)

    # 레이아웃 생성 및 위젯에 추가
    layout = QVBoxLayout()
    layout.addWidget(stacked_widget)
    window.setLayout(layout)
    window.setWindowFlag(Qt.WindowTitleHint, False)

    window.showMaximized()

    sys.exit(app.exec())
