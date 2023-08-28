import sys
# tell python where to find libraries when running as superuser
sys.path.append('/home/rover_pi/.local/lib/python3.9/site-packages')

import cv2
from cv2 import VideoWriter
from flask import Flask, render_template, request, redirect, url_for, Response
import time
import os
import shutil
from datetime import datetime, timedelta
from threading import Thread
from pymavlink import mavutil
import csv
from PositioningSystem.yolov5.detect import ObjectDetector


app = Flask(__name__)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 1
MAVstatus = 'Disconnected'
Datalogging = False
pdir = './logs/'
detector = ObjectDetector()

def connectMAV():
    global master
    global MAVstatus
    global btn1

    # Create the connection
    # Need to provide the serial port and baudrate
    master = mavutil.mavlink_connection(
        "/dev/ttyUSB0", baud=115200)  # usb on windows
    # find the OSAVC controller
    master.wait_heartbeat(timeout=5)
    if (MAVstatus == 'Disconnected'):
        if master.target_system == 0:
            print('No system detected!')
            MAVstatus = 'Timeout'
        else:
            print('target_system {}, target component {} \n'.format(
                master.target_system, master.target_component))
            btn1 = 'h'
            MAVstatus = 'Connected'
    print('connectMAV thread stopped')


def MAVlogging():
    global master
    global MAVstatus
    global btn1
    t = '{:%Y%m%d-%H%M%S}'.format(datetime.now())
    dirname = pdir + '/logfiles/'
    csv_file = dirname+ 'log_' + t + '.csv'
    os.makedirs(dirname, exist_ok=True)

# first find all the incoming messages:
    msgs_dict = {}
    start_time = time.time()
    end_time = 5
    while time.time() - start_time < end_time:
        msg = master.recv_match(blocking=True)
        if not msg:
            MAVstatus = 'No Data Found!'
        if msg.get_type() == "BAD_DATA":
            pass
            """ if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush() """
        else:
            # Message is valid
            # Use the attribute
            msgs_dict.update(msg.to_dict())
    # Put all  keys for all the incoming messages into the headers list
    headers = list(msgs_dict)
    print(headers)
    with open(csv_file, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=headers)
        writer.writeheader()
        start_time = time.time()
        logging_time = 20
        # currently we log for a specified period of time
        while Datalogging == True and time.time() - start_time < logging_time:
            msg = master.recv_match(blocking=True)
            if not msg:
                MAVstatus = 'No Data Found!'
            if msg.get_type() == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    pass
            else:
                # add msg to the msgs_dict
                msgs_dict.update(msg.to_dict())
                # and write it to the file
                writer.writerow(msgs_dict)
    # finish up:
    master.close()
    print('Exiting datalogging script')


def timelapse():  # continuous shooting
    # open webcam
    cam = cv2.VideoCapture(-1, cv2.CAP_V4L)
    print('timelapse mode on')
    dirname = pdir + '/images/'
    os.makedirs(dirname, exist_ok=True)
    while btn1 == 's':
        t = '{:%Y%m%d-%H%M%S}'.format(datetime.now())
        filename = dirname+'img'+t+'.jpg'
        stream_ok, frame = cam.read()
        if stream_ok:
            cv2.imwrite(filename, frame)
        print('snap taken')
        print(btn1, btn2)
        shutil.copyfile(filename,'./static/latest.jpg')
    cam.release()
    print('timelapse mode off') 


def video():  # record video stream
    # open webcam
    cam = cv2.VideoCapture(-1, cv2.CAP_V4L)
    # define video codec
    videoCodec = VideoWriter.fourcc(*'MJPG')
    t = '{:%Y%m%d-%H%M%S}'.format(datetime.now())
    # create video file and store on USB drive
    dirname = pdir + '/video/'
    os.makedirs(dirname, exist_ok=True)
    video = VideoWriter(dirname+'vid' + t + '.avi', videoCodec, 10/1, (int(cam.get(3)), int(cam.get(4))))
    while btn1 == 'v':
        print(btn1, btn2)
        # read camera frame
        stream_ok, frame = cam.read()
        if stream_ok:
            video.write(frame)
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    # clean up streams
    cam.release()
    video.release()
    print('video thread stopped')

@app.route('/video_feed') 
def video_feed(): 
   """Video streaming route. Put this in the src attribute of an img tag.""" 
   return Response(video(), 
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detect_feed') 
def detect_feed():
   if btn1 == 'd':
       return Response(detector.run_generator(weights='./PositioningSystem_raspi/weights/bestv4-int8_edgetpu.tflite'), 
                       mimetype='multipart/x-mixed-replace; boundary=frame')
   
def snapstart():  # take pictures on demand
    cam = cv2.VideoCapture(-1, cv2.CAP_V4L)
    print('entered snapshot mode')
    global btn2
    dirname = pdir + '/images/'
    os.makedirs(dirname, exist_ok=True)
    while btn1 == 'q':
        time.sleep(0.1)
        if btn2 == 'a':
            print('taken snap: btn2 =' + btn2)
            t = '{:%Y%m%d-%H%M%S}'.format(datetime.now())
            filename = dirname+'img'+t+'.jpg'
            stream_ok, frame = cam.read()
            if stream_ok:
                cv2.imwrite(filename, frame)
                shutil.copyfile(filename, './static/latest.jpg')
            btn2 = 'o'
            print('btn2 =' + btn2)
    # clean up stream
    cam.release()
    print('exiting snaphot mode')

# we are able to make two different requests on our webpage
# GET = we just type in the url
# POST = some sort of form submission like a button

@app.route('/', methods=['POST', 'GET'])
def hello_world():

    status = 'off'
    global btn1
    btn1 = 'o'
    global btn2
    btn2 = 'o'
    message = 'All good'
    global MAVstatus
    global Datalogging
    global pdir

    # if we make a post request on the webpage aka press button then do stuff
    if request.method == 'POST':
        # if we press the turn on button
        if request.form['submit'] == 'Enter directory':
            pdir = request.form['projectDir']
            print(pdir)
        elif request.form['submit'] == 'Video':
            print('BP: Recording video')
            status = 'video'
            btn1 = 'v'
            # t2 = Thread(target=video)
            # t2.start()
            message = 'All good'
        elif request.form['submit'] == 'Video Off':
            print('BP: Video off')
            status = 'Idle'
            btn1 = 'o'
            message = 'All good'
        elif request.form['submit'] == 'Connect MAV':
            print('Trying to connect')
            if MAVstatus == 'Disconnected':
                status = 'Connecting to MAV'
            btn1 = 'h'
            t4 = Thread(target=connectMAV)
            t4.start()
        elif request.form['submit'] == 'Start logging':
            Datalogging = True
            btn1 = 'datalog'
            t5 = Thread(target=MAVlogging)
            t5.start()
        elif request.form['submit'] == 'Stop logging':
            Datalogging = False
            btn1 = 'o'
        elif request.form['submit'] == 'Stills':
            print('BP: Recording stills')
            btn1 = 's'
            t1 = Thread(target=timelapse)
            t1.start()
            status = 'stills'
            message = 'All good'
        elif request.form['submit'] == 'Stills Off':
            print('BP: stills off')
            status = 'Idle'
            btn1 = 'o'
            message = 'All good'
        elif request.form['submit'] == 'QuickSnap':
            print('BP: QuickSnap')
            status = 'Ready to snap'
            btn1 = 'q'
            t3 = Thread(target=snapstart)
            t3.start()
            message = 'All good'
        elif request.form['submit'] == 'QuickSnap Off':
            print('BP:QuickSnap off')
            status = 'Idle'
            btn1 = 'o'
            message = 'All good'
        elif request.form['submit'] == 'Take':
            print('BP:Take')
            status = 'Snapshot mode'
            btn1 = 'q'
            btn2 = 'a'
            message = 'All good'
        elif request.form['submit'] == '_Take_':
            print('BP:Take error')
            status = 'Error'
            message = 'Enable QuickSnap first'
            btn1 = 'o'
        elif request.form['submit'] == 'Detect':
            print('BP: Running detection')
            status = 'Detect mode'
            btn1 = 'd'
            # t6 = Thread(target=detect_feed)
            # t6.start()
            message = 'All good'
        elif request.form['submit'] == 'Detect Off':
            print('BP: Detect off')
            detector.stop()
            status = 'Idle'
            btn1 = 'o'
            message = 'All good'
        else:
            pass

#    temp = round(bme280.get_temperature(),2) # temperature
    temp = 0
#    press = int(bme280.get_pressure()) # pressure
    press = 0
#    lux = ltr559.get_lux() # light levels
    lux = 0
    df = os.statvfs('/')  # check if we're running out of disk space
    df_size = df.f_frsize * df.f_blocks
    df_avail = df.f_frsize * df.f_bfree
    df_pc = round((100 * df_avail/df_size), 1)
    print(btn1, btn2)

    # the default page to display will be our template with our template variables
    return render_template('index2.html', MAVstatus=MAVstatus, message=message, status=status, temp=temp, press=press, lux=lux, df_pc=df_pc, btn1= btn1)


if __name__ == "__main__":

    # let's launch our webpage!
    # do 0.0.0.0 so that we can log into this webpage
    # using another computer on the same network later
    app.run(host='0.0.0.0', port=5000, debug=True)
