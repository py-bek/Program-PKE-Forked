"""
Drone delivery:

we are going to build a mission in mission planner, upload the mission to the drone.

The script will connect with the vehicle and check if a new mission has been uploaded.
As soon as a valid mission is available, we takeoff in GUIDED mode and then we switch
to AUTO.
When the mission is completed we command to co back to home and land
"""



try:
    from collections import abc

    print("READY")

    collections.MutableMapping = abc.MutableMapping

except:
    pass

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil


# --------------------------------------------------
# -------------- CONNECTION
# --------------------------------------------------
# -- Connect to the vehicle
print('Connecting...')
vehicle = connect('tcp:127.0.0.1:5762')


# vehicle = connect('/dev/ttyACM0')
# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

# --------------------------------------------------
# -------------- FUNCTIONS
# --------------------------------------------------
# -- Define arm and takeoff
def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")

    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
        time.sleep(1)


def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.flush()

    # After clearing the mission you MUST re-download the mission from the vehicle
    # before vehicle.commands can be used again
    # (see https://github.com/dronekit/dronekit-python/issues/230)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()


def download_mission(vehicle):
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()  # wait until download is complete.


def get_current_mission(vehicle):
    """
    Downloads the mission and returns the wp list and number of WP

    Input:
        vehicle

    Return:
        n_wp, wpList
    """

    print("Downloading mission")
    download_mission(vehicle)
    missionList = []
    n_WP = 0
    for wp in vehicle.commands:
        missionList.append(wp)
        n_WP += 1

    return n_WP, missionList


def add_last_waypoint_to_mission(  # --- Adds a last waypoint on the current mission file
        vehicle,  # --- vehicle object
        wp_Last_Latitude,  # --- [deg]  Target Latitude
        wp_Last_Longitude,  # --- [deg]  Target Longitude
        wp_Last_Altitude):  # --- [m]    Target Altitude
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
    # Get the set of commands from the vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # Save the vehicle commands to a list
    missionlist = []
    for cmd in cmds:
        missionlist.append(cmd)

    # Modify the mission as needed. For example, here we change the
    wpLastObject = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                           0, 0, 0, 0, 0, 0,
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    # Clear the current mission (command is sent when we call upload())
    cmds.clear()

    # Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()

    return (cmds.count)


def mundur():
    print("mundur")
    vehicle.message_factory.set_position_target_local_ned_send(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b100111100111,  # 0b0000111111000111,  # Set velocity components
        0, 0, 0,
        -0.5, 0, 0,
        0, 0, 0,
        0, 0
    )


def stop():
    print("stop")
    vehicle.message_factory.set_position_target_local_ned_send(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b100111100111,  # 0b0000111111000111,  # Set velocity components
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def gerak(vx, vy):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, 0,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def ambil_foto():
    cam_port = 0
    # cam = cv2.VideoCapture(cam_port)

    # reading the input using the camera
    # result, image = cam.read()
    # cv2.imshow("Bawah Air", image)
    # cv2.imwrite("Bawah_Air.png", image)
    # cv2.waitKey(0)
    # cv2.destroyWindow("Bawah Air")


def ChangeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
        vehicle.mode = VehicleMode(mode)
        time.sleep(0.5)
    return True



lurus = 0
s1 = 2.5
s2 = 5
s3 = 7.5
s4 = 10
s5 = 15
s6 = 22.5
s7 = 30
s8 = 37.5
s9 = 45
s10 = 52.5
s11 = 60


def sudutBelokY(sdt, speed):
    return float("{:.2f}".format(math.sin(math.radians(sdt)) * speed))


def sudutBelokX(sdt, speed):
    return float("{:.2f}".format(math.cos(math.radians(sdt)) * speed))


def greenBall(xh, yh):
    if 120 <= xh < 220 and 280 <= yh < 330:
        return 'e'
    elif 220 <= xh < 320 and 280 <= yh < 330:
        return 'e'
    elif 320 <= xh < 420 and 280 <= yh < 330:
        return 'c'
    elif 420 <= xh < 520 and 280 <= yh < 330:
        return 'a'
    elif 100 <= xh < 210 and 330 <= yh < 380:
        return 'f'
    elif 210 <= xh < 320 and 330 <= yh < 380:
        return 'f'
    elif 320 <= xh < 430 and 330 <= yh < 380:
        return 'e'
    elif 430 <= xh < 540 and 330 <= yh < 380:
        return 'b'
    elif 80 <= xh < 200 and 380 <= yh < 430:
        return 'g'
    elif 200 <= xh < 320 and 380 <= yh < 430:
        return 'g'
    elif 320 <= xh < 440 and 380 <= yh < 430:
        return 'f'
    elif 440 <= xh < 560 and 380 <= yh < 430:
        return 'd'
    elif 60 <= xh < 190 and 430 <= yh < 480:
        return 'h'
    elif 190 <= xh < 320 and 430 <= yh < 480:
        return 'g'
    elif 320 <= xh < 450 and 430 <= yh < 480:
        return 'f'
    elif 450 <= xh < 580 and 430 <= yh < 480:
        return 'd'
    else:
        return 'none'


def redBall(xm, ym):
    if 120 <= xm < 220 and 280 <= ym < 330:
        return 'A'
    elif 220 <= xm < 320 and 280 <= ym < 330:
        return 'C'
    elif 320 <= xm < 420 and 280 <= ym < 330:
        return 'E'
    elif 420 <= xm < 520 and 280 <= ym < 330:
        return 'E'
    elif 100 <= xm < 210 and 330 <= ym < 380:
        return 'B'
    elif 210 <= xm < 320 and 330 <= ym < 380:
        return 'E'
    elif 320 <= xm < 430 and 330 <= ym < 380:
        return 'F'
    elif 430 <= xm < 540 and 330 <= ym < 380:
        return 'F'
    elif 80 <= xm < 200 and 380 <= ym < 430:
        return 'D'
    elif 200 <= xm < 320 and 380 <= ym < 430:
        return 'F'
    elif 320 <= xm < 440 and 380 <= ym < 430:
        return 'G'
    elif 440 <= xm < 560 and 380 <= ym < 430:
        return 'G'
    elif 60 <= xm < 190 and 430 <= ym < 480:
        return 'D'
    elif 190 <= xm < 320 and 430 <= ym < 480:
        return 'F'
    elif 320 <= xm < 450 and 430 <= ym < 480:
        return 'G'
    elif 450 <= xm < 580 and 430 <= ym < 480:
        return 'H'
    else:
        return 'none'


def kondisi(mr, mg, speed):
    if len(mr) != 0 and len(mg) == 0:
        if mr == 'C':
            print('R6')
            print(sudutBelokX(s6, speed), sudutBelokY(s6, speed))
            gerak(sudutBelokX(s6, speed), sudutBelokY(s6, speed))
        elif mr == 'E':
            print('R8')
            print(sudutBelokX(s8, speed), sudutBelokY(s8, speed))
            gerak(sudutBelokX(s8, speed), sudutBelokY(s8, speed))
        elif mr == 'F':
            print('R9')
            print(sudutBelokX(s9, speed), sudutBelokY(s9, speed))
            gerak(sudutBelokX(s9, speed), sudutBelokY(s9, speed))
        else:
            print('R11')
            print(sudutBelokX(s11, speed), sudutBelokY(s11, speed))
            gerak(sudutBelokX(s11, speed), sudutBelokY(s11, speed))
    elif len(mr) == 0 and len(mg) != 0:
        if mg == 'c':
            print('L6')
            print(sudutBelokX(-s6, speed), sudutBelokY(-s6, speed))
            gerak(sudutBelokX(-s6, speed), sudutBelokY(-s6, speed))
        elif mg == 'e':
            print('L8')
            print(sudutBelokX(-s8, speed), sudutBelokY(-s8, speed))
            gerak(sudutBelokX(-s8, speed), sudutBelokY(-s8, speed))
        elif mg == 'f':
            print('L9')
            print(sudutBelokX(-s9, speed), sudutBelokY(-s9, speed))
            gerak(sudutBelokX(-s9, speed), sudutBelokY(-s9, speed))
        else:
            print('L11')
            print(sudutBelokX(-s11, speed), sudutBelokY(-s11, speed))
            gerak(sudutBelokX(-s11, speed), sudutBelokY(-s11, speed))
    elif len(mr) != 0 and len(mg) != 0:
        if (mr == 'A' and mg == 'a') or (mr == 'B' and mg == 'b') or (mr == 'D' and mg == 'd'):
            print('LURUS')
            print(sudutBelokX(lurus, speed), sudutBelokY(lurus, speed))
            gerak(sudutBelokX(lurus, speed), sudutBelokY(lurus, speed))
        elif mr == 'A' and mg == 'c':
            print('L2')
            print(sudutBelokX(-s2, speed), sudutBelokY(-s2, speed))
            gerak(sudutBelokX(-s2, speed), sudutBelokY(-s2, speed))
        elif mr == 'C' and mg == 'a':
            print('R2')
            print(sudutBelokX(s2, speed), sudutBelokY(s2, speed))
            gerak(sudutBelokX(s2, speed), sudutBelokY(s2, speed))
        elif mr == 'A' and mg == 'b':
            print('R2')
            print(sudutBelokX(s2, speed), sudutBelokY(s2, speed))
            gerak(sudutBelokX(s2, speed), sudutBelokY(s2, speed))
        elif mr == 'A' and mg == 'e':
            print('L3')
            print(sudutBelokX(-s3, speed), sudutBelokY(-s3, speed))
            gerak(sudutBelokX(-s3, speed), sudutBelokY(-s3, speed))
        elif mr == 'E' and mg == 'a':
            print('R3')
            print(sudutBelokX(s3, speed), sudutBelokY(s3, speed))
            gerak(sudutBelokX(s3, speed), sudutBelokY(s3, speed))
        elif mr == 'A' and mg == 'd':
            print('R4')
            print(sudutBelokX(s4, speed), sudutBelokY(s4, speed))
            gerak(sudutBelokX(s4, speed), sudutBelokY(s4, speed))
        elif mr == 'A' and mg == 'f':
            print('L5')
            print(sudutBelokX(-s5, speed), sudutBelokY(-s5, speed))
            gerak(sudutBelokX(-s5, speed), sudutBelokY(-s5, speed))
        elif mr == 'F' and mg == 'a':
            print('R5')
            print(sudutBelokX(s5, speed), sudutBelokY(s5, speed))
            gerak(sudutBelokX(s5, speed), sudutBelokY(s5, speed))
        elif mr == 'B' and mg == 'a':
            print('L2')
            print(sudutBelokX(-s2, speed), sudutBelokY(-s2, speed))
            gerak(sudutBelokX(-s2, speed), sudutBelokY(-s2, speed))
        elif mr == 'B' and mg == 'c':
            print('L2')
            print(sudutBelokX(-s2, speed), sudutBelokY(-s2, speed))
            gerak(sudutBelokX(-s2, speed), sudutBelokY(-s2, speed))
        elif mr == 'C' and mg == 'b':
            print('R2')
            print(sudutBelokX(s2, speed), sudutBelokY(s2, speed))
            gerak(sudutBelokX(s2, speed), sudutBelokY(s2, speed))
        elif mr == 'B' and mg == 'e':
            print('L1')
            print(sudutBelokX(-s1, speed), sudutBelokY(-s1, speed))
            gerak(sudutBelokX(-s1, speed), sudutBelokY(-s1, speed))
        elif mr == 'E' and mg == 'b':
            print('R1')
            print(sudutBelokX(s1, speed), sudutBelokY(s1, speed))
            gerak(sudutBelokX(s1, speed), sudutBelokY(s1, speed))
        elif mr == 'B' and mg == 'd':
            print('L3')
            print(sudutBelokX(-s3, speed), sudutBelokY(-s3, speed))
            gerak(sudutBelokX(-s3, speed), sudutBelokY(-s3, speed))
        elif mr == 'B' and mg == 'f':
            print('L4')
            print(sudutBelokX(-s4, speed), sudutBelokY(-s4, speed))
            gerak(sudutBelokX(-s4, speed), sudutBelokY(-s4, speed))
        elif mr == 'F' and mg == 'b':
            print('R4')
            print(sudutBelokX(s4, speed), sudutBelokY(s4, speed))
            gerak(sudutBelokX(s4, speed), sudutBelokY(s4, speed))
        elif mr == 'D' and mg == 'a':
            print('R4')
            print(sudutBelokX(s4, speed), sudutBelokY(s4, speed))
            gerak(sudutBelokX(s4, speed), sudutBelokY(s4, speed))
        elif mr == 'D' and mg == 'c':
            print('L4')
            print(sudutBelokX(-s4, speed), sudutBelokY(-s4, speed))
            gerak(sudutBelokX(-s4, speed), sudutBelokY(-s4, speed))
        elif mr == 'C' and mg == 'd':
            print('R4')
            print(sudutBelokX(s4, speed), sudutBelokY(s4, speed))
            gerak(sudutBelokX(s4, speed), sudutBelokY(s4, speed))
        elif mr == 'D' and mg == 'b':
            print('L2')
            print(sudutBelokX(-s2, speed), sudutBelokY(-s2, speed))
            gerak(sudutBelokX(-s2, speed), sudutBelokY(-s2, speed))
        elif mr == 'D' and mg == 'e':
            print('L5')
            print(sudutBelokX(-s5, speed), sudutBelokY(-s5, speed))
            gerak(sudutBelokX(-s5, speed), sudutBelokY(-s5, speed))
        elif mr == 'E' and mg == 'd':
            print('R5')
            print(sudutBelokX(s5, speed), sudutBelokY(s5, speed))
            gerak(sudutBelokX(s5, speed), sudutBelokY(s5, speed))
        elif mr == 'D' and mg == 'f':
            print('L6')
            print(sudutBelokX(-s6, speed), sudutBelokY(-s6, speed))
            gerak(sudutBelokX(-s6, speed), sudutBelokY(-s6, speed))
        elif mr == 'F' and mg == 'd':
            print('R6')
            print(sudutBelokX(s6, speed), sudutBelokY(s6, speed))
            gerak(sudutBelokX(s6, speed), sudutBelokY(s6, speed))
        elif mr == 'C' and mg == 'f':
            print('L4')
            print(sudutBelokX(-s4, speed), sudutBelokY(-s4, speed))
            gerak(sudutBelokX(-s4, speed), sudutBelokY(-s4, speed))
        elif mr == 'F' and mg == 'c':
            print('R4')
            print(sudutBelokX(s4, speed), sudutBelokY(s4, speed))
            gerak(sudutBelokX(s4, speed), sudutBelokY(s4, speed))
    elif len(mr) == 0 and len(mg) == 0:
        print('LURUS')
        print(sudutBelokX(lurus, speed), sudutBelokY(lurus, speed))
        gerak(sudutBelokX(lurus, speed), sudutBelokY(lurus, speed))

def MISI1(vehicle):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    model = torch.hub.load('ultralytics/yolov5', 'custom', 'dataset/yolov5n_ds2.pt')

    fps_start_time = datetime.now()
    fps = 0
    total_frames = 1

    classNames = model.names

    colors = [(0, 255, 0), (0, 0, 255), (255, 0, 0), (0, 255, 255), (255, 255, 0)]

    membersGreen = []
    membersRed = []

    speed = 0.5

    while True:
        success, frame = cap.read()
        if not success:
            break

        # grsh1 = cv2.line(frame, (120, 280), (520, 280), (0, 0, 0), 2)
        # grsh2 = cv2.line(frame, (100, 330), (540, 330), (0, 0, 0), 2)
        # grsh3 = cv2.line(frame, (80, 380), (560, 380), (0, 0, 0), 2)
        # grsh4 = cv2.line(frame, (60, 430), (580, 430), (0, 0, 0), 2)
        
        # grsv1 = cv2.line(frame, (120, 280), (40, 480), (0, 0, 0), 2)
        # grsv2 = cv2.line(frame, (220, 280), (180, 480), (0, 0, 0), 2)
        # grsv3 = cv2.line(frame, (320, 280), (320, 480), (0, 0, 0), 2)
        # grsv4 = cv2.line(frame, (420, 280), (460, 480), (0, 0, 0), 2)
        # grsv5 = cv2.line(frame, (520, 280), (600, 480), (0, 0, 0), 2)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # frame = cv2.resize(frame, (640, 640))
        model.conf = 0.6
        model.iou = 0.4
        results = model(frame)

        total_frames += 1
        fps_text = "FPS: {:.2f}".format(fps)
        # print(fps_text)
        cv2.putText(frame, fps_text, (5, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 1)

        for det in results.pred[0]:
            box, class_id, confidence = det[:4], int(det[5]), det[4]

            x1, y1, x2, y2 = map(int, box)
            color = colors[class_id % len(colors)]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

            rmsx = x1 + ((x2 - x1) / 2)
            # print("x =", RMSX, class_id)
            rmsy = y1 + ((y2 - y1) / 2)
            # print("y =", RMSY, class_id)
            
            cond = redBall(rmsx, rmsy)
            if cond != 'none' and class_id == 1:
                membersRed.append(cond)

            cond = greenBall(rmsx, rmsy)
            if cond != 'none' and class_id == 0:
                membersGreen.append(cond)

            org = (x1, y1 - 10)
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            cv2.putText(frame, f"{classNames[class_id]} ({confidence:.2f})", org, font, fontScale, color, 2)

        membersRed.sort()
        membersGreen.sort()

        mR = ''
        while membersRed:
            mR = membersRed.pop(0)

        mG = ''
        while membersGreen:
            mG = membersGreen.pop(0)

        # print(mR)
        # print(mG)

        kondisi(mR, mG, speed)

        cv2.imshow('Webcam', frame)
        fps_end_time = datetime.now()
        time_diff = fps_end_time - fps_start_time
        fps = total_frames / time_diff.total_seconds()

        current_loc_lat = vehicle.location.global_relative_frame.lat
        current_loc_lon = vehicle.location.global_relative_frame.lon
        print(current_loc_lat, current_loc_lon)

        if cv2.waitKey(1) == ord('q'):
            break

        # if (current_loc_lat <= -35.3609511 and current_loc_lon <= 149.1690952) and (current_loc_lat >= -35.3610862 and current_loc_lon >= 149.1690000 ) :
        #     print("STOP PROGRAM MISI 1")
        #     break

        # (-y > a > -y-500) && (x+200 > b > x-200)
        if ( -7.5613886 >= current_loc_lat >= -7.5614086 and  110.8575823 <= current_loc_lon <= 110.8576023) :
            print("STOP PROGRAM MISI 1")
            break
        
        if ( -7.5614368 >= current_loc_lat >= -7.5614368 and  110.8575823 >= current_loc_lon >= 110.8576023) :
            print("STOP PROGRAM MISI 3")
            break


    cap.release()
    cv2.destroyAllWindows()

def MISI2(vehicle):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    model = torch.hub.load('ultralytics/yolov5', 'custom', 'dataset/yolov5n_ds2.pt')

    fps_start_time = datetime.now()
    fps = 0
    total_frames = 1

    classNames = model.names

    colors = [(0, 255, 0), (0, 0, 255), (255, 0, 0), (0, 255, 255), (255, 255, 0)]

    membersGreen = []
    membersRed = []

    speed = 0.5

    while True:
        success, frame = cap.read()
        if not success:
            break

        # grsh1 = cv2.line(frame, (120, 280), (520, 280), (0, 0, 0), 2)
        # grsh2 = cv2.line(frame, (100, 330), (540, 330), (0, 0, 0), 2)
        # grsh3 = cv2.line(frame, (80, 380), (560, 380), (0, 0, 0), 2)
        # grsh4 = cv2.line(frame, (60, 430), (580, 430), (0, 0, 0), 2)
        #
        # grsv1 = cv2.line(frame, (120, 280), (40, 480), (0, 0, 0), 2)
        # grsv2 = cv2.line(frame, (220, 280), (180, 480), (0, 0, 0), 2)
        # grsv3 = cv2.line(frame, (320, 280), (320, 480), (0, 0, 0), 2)
        # grsv4 = cv2.line(frame, (420, 280), (460, 480), (0, 0, 0), 2)
        # grsv5 = cv2.line(frame, (520, 280), (600, 480), (0, 0, 0), 2)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # frame = cv2.resize(frame, (640, 640))
        model.conf = 0.6
        model.iou = 0.4
        results = model(frame)

        total_frames += 1
        fps_text = "FPS: {:.2f}".format(fps)
        # print(fps_text)
        cv2.putText(frame, fps_text, (5, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 1)

        for det in results.pred[0]:
            box, class_id, confidence = det[:4], int(det[5]), det[4]

            x1, y1, x2, y2 = map(int, box)
            color = colors[class_id % len(colors)]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

            rmsx = x1 + ((x2 - x1) / 2)
            # print("x =", RMSX, class_id)
            rmsy = y1 + ((y2 - y1) / 2)
            # print("y =", RMSY, class_id)

            cond = redBall(rmsx, rmsy)
            if cond != 'none' and class_id == 1:
                membersRed.append(cond)

            cond = greenBall(rmsx, rmsy)
            if cond != 'none' and class_id == 0:
                membersGreen.append(cond)

            org = (x1, y1 - 10)
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            cv2.putText(frame, f"{classNames[class_id]} ({confidence:.2f})", org, font, fontScale, color, 2)

        membersRed.sort()
        membersGreen.sort()

        mR = ''
        while membersRed:
            mR = membersRed.pop(0)

        mG = ''
        while membersGreen:
            mG = membersGreen.pop(0)

        # print(mR)
        # print(mG)

        kondisi(mG, mR, speed)

        cv2.imshow('Webcam', frame)
        fps_end_time = datetime.now()
        time_diff = fps_end_time - fps_start_time
        fps = total_frames / time_diff.total_seconds()

        current_loc_lat = vehicle.location.global_relative_frame.lat
        current_loc_lon = vehicle.location.global_relative_frame.lon
        print(current_loc_lat, current_loc_lon)

        if cv2.waitKey(1) == ord('q'):
            break

        if (-7.5614785 >= current_loc_lat >= -7.5614885 and  110.8575727 <= current_loc_lon <= 110.8575827)  :
            print("STOP PROGRAM")
            break


    cap.release()
    cv2.destroyAllWindows()


# -----------0-------------------------------------
# -----------0- INITIALIZE
# -----------0-------------------------------------
# -- Setup th0commanded flying speed
gnd_speed = 300# [m/s]
mode = 'GROUND'


# --------------------------------------------------
# -------------- MAIN FUNCTION
# --------------------------------------------------
while True:

    if mode == 'GROUND':
        # --- Wait until a valid mission has been uploaded
        n_WP, missionList = get_current_mission(vehicle)
        # nwp = vehicle.commands.next
        time.sleep(2)
        if n_WP > 0:
            print("A valid mission has been uploaded: takeoff!")
            mode = 'TAKEOFF'

    elif mode == 'TAKEOFF':

        # -- Add a fake waypoint at the end of the mission
        add_last_waypoint_to_mission(vehicle, vehicle.location.global_relative_frame.lat,
                                     vehicle.location.global_relative_frame.lon,
                                     vehicle.location.global_relative_frame.alt)
        print("Home waypoint added to the mission")
        time.sleep(1)
        # -- Takeoff
        arm_and_takeoff(0)

        # -- Change the UAV mode to AUTO
        print("Changing to AUTO")
        ChangeMode(vehicle, "AUTO")

        # -- Change mode, set the ground speed
        vehicle.groundspeed = gnd_speed
        mode = 'MISSION'
        print("Swiitch mode to MISSION")

    elif mode == 'MISSION':
        # -- Here we just monitor the mission status. Once the mission is completed we go back
        # -- vehicle.commands.cout is the total number of waypoints
        # -- vehicle.commands.next is the waypoint the vehicle is going to
        # -- once next == cout, we just go home
        nwp = vehicle.commands.next
        awp = vehicle.commands.count
        print("Current WP: %d of %d " % (vehicle.commands.next, vehicle.commands.count))
        if nwp == 1:
            print("WARNING")
            ChangeMode(vehicle, "AUTO")
            time.sleep(3)
            while True:
                print("MODE AUTO")
                nwp = vehicle.commands.next
                print("NWP : ", nwp)
                if nwp == 2:
                    break
            print("OUT AUTO")

        if nwp == 2:
            print("MODE GUIDED MISI 1")
            ChangeMode(vehicle, "GUIDED")
            time.sleep(3)
            print(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
            MISI1(vehicle)
            print("OUT GUIDED")
            nwp = 3

        if nwp == 3:
            print("WARNING")
            ChangeMode(vehicle, "AUTO")
            time.sleep(3)
            while True:
                print("MODE AUTO")
                nwp = vehicle.commands.next
                print("NWP : ", nwp)
                if nwp == 5:
                    break

            print("OUT AUTO")

        if nwp == 5:
            print("MODE GUIDED MISI 2")
            ChangeMode(vehicle, "GUIDED")
            time.sleep(3)
            print(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
            MISI2(vehicle)
            print("OUT GUIDED")
            nwp = 6

        if nwp == 6:
            print("WARNING")
            ChangeMode(vehicle, "AUTO")
            time.sleep(3)
            while True:
                print("MODE AUTO")
                nwp = vehicle.commands.next
                print("NWP : ", nwp)
                if nwp == 8:
                    break

            print("OUT AUTO")

        if nwp == 8:
            print("MODE GUIDED MISI 3")
            ChangeMode(vehicle, "GUIDED")
            time.sleep(3)
            print(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
            MISI1(vehicle)
            print("OUT GUIDED")

        if nwp != awp:
            print("WARNING")
            ChangeMode(vehicle, "AUTO")
            time.sleep(3)
            while True:
                print("MODE AUTO")
                nwp = vehicle.commands.next
                print("NWP : ", nwp)
                if nwp == awp:
                    break

            print("OUT AUTO")

        if nwp == awp:
            ChangeMode(vehicle, "RTL")
            print("MISSION = RTL")
            clear_mission(vehicle)
            print("DELETING MISSION")
            mode = "BACK"

    elif mode == "BACK":
        if vehicle.location.global_relative_frame.alt < 1:
            print("Switch to GROUND mode, waiting for new missions")
            mode = 'GROUND'
            break
