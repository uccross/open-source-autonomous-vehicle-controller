from doctest import master
from flask import Flask, render_template,request, redirect, url_for
import time, os, shutil
from picamera import PiCamera
from datetime import datetime, timedelta
from threading import Thread
from pymavlink import mavutil
import csv
import sys


app = Flask(__name__)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 1
MAVstatus = 'Disconnected'
Datalogging = False


def connectMAV():
    global master
    global MAVstatus
    global btn1
 
    # Create the connection
    # Need to provide the serial port and baudrate
    master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=115200) # usb on windows
    # find the OSAVC controller
    master.wait_heartbeat(timeout = 5)
    if(MAVstatus == 'Disconnected'):
        if master.target_system == 0:
            print('No system detected!')
            MAVstatus = 'Timeout'
        else:
            print('target_system {}, target component {} \n'.format(master.target_system,master.target_component))
            btn1 = 'h'
            MAVstatus = 'Connected'
    print('connectMAV thread stopped')

def MAVlogging():
    global master
    global MAVstatus
    global btn1
    t='{:%Y%m%d-%H%M%S}'.format(datetime.now())
    csv_file = '/mnt/usb/logfiles/log_' + t + '.csv'
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
            #Message is valid
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
            msg = master.recv_match(blocking = True)
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
    cam = PiCamera()
    cam.resolution = (1640,922)
    for filename in enumerate(cam.capture_continuous('/mnt/usb/images/img{timestamp:%Y%m%d-%H%M%S}.jpg')):
        print('snap taken')
        print(btn1,btn2)
        print(filename)
        #shutil.copyfile(filename,'/mnt/usb/images/filename')
        #shutil.copyfile(filename,'/home/pi/Flask/static/latest.jpg')
        if btn1 != 's':
            break
    cam.close()
    print('timelapse thread stopped')

def video(): # record a video
    cam = PiCamera()
    t='{:%Y%m%d-%H%M%S}'.format(datetime.now())
    cam.resolution = (640,480)
    cam.start_recording('/mnt/usb/video/vid'+t+'.h264')
    while btn1 == 'v':
        print(btn1,btn2)
        pass
    cam.stop_recording()
    cam.close()
    print('video thread stopped')

def snapstart(): # take pictures on demand
    cam = PiCamera()
    cam.resolution = (1640,922)
    print('entered snapshot mode')
    global btn2
    while btn1 == 'q':
        time.sleep(0.1)
        if btn2 == 'a':
            print('taken snap: btn2 =' + btn2)
            t='{:%Y%m%d-%H%M%S}'.format(datetime.now())
            filename = '/mnt/usb/images/img'+t+'.jpg'
            cam.capture(filename)
            shutil.copyfile(filename,'/mnt/usb/latest/latest.jpg')
            shutil.copyfile(filename,'/home/pi/Flask/static/latest.jpg')
            btn2 = 'o'
            print('btn2 =' + btn2)

    cam.close()
    print('exiting snaphot mode')


# we are able to make two different requests on our webpage
# GET = we just type in the url
# POST = some sort of form submission like a button

@app.route('/', methods = ['POST','GET'])
def hello_world():

    
    status = 'off'
    global btn1
    btn1 = 'o'
    global btn2
    btn2 = 'o'
    message = 'All good'
    global MAVstatus
    global Datalogging

    # if we make a post request on the webpage aka press button then do stuff
    if request.method == 'POST':

        # if we press the turn on button
        if request.form['submit'] == 'Video':
            print('BP: Recording video')
            status = 'video'
            btn1 = 'v'
            t2 = Thread(target=video)
            t2.start()
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
        else:
            pass

#    temp = round(bme280.get_temperature(),2) # temperature
    temp = 0
#    press = int(bme280.get_pressure()) # pressure
    press = 0
#    lux = ltr559.get_lux() # light levels
    lux = 0
    df = os.statvfs('/') # check if we're running out of disk space
    df_size = df.f_frsize * df.f_blocks
    df_avail = df.f_frsize * df.f_bfree
    df_pc = round((100 * df_avail/df_size),1)
    print(btn1, btn2)

    # the default page to display will be our template with our template variables
    return render_template('index2.html', MAVstatus= MAVstatus, message= message, status=status, temp=temp, press=press, lux=lux, df_pc=df_pc, btn1 = btn1)

if __name__ == "__main__":

    # let's launch our webpage!
    # do 0.0.0.0 so that we can log into this webpage
    # using another computer on the same network later
    # specify port 80 rather than default 5000
    app.run(host='0.0.0.0',port=5000,debug=True)
