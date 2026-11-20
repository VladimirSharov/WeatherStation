import serial, sys
import csv
import time
import datetime
import random
import MySQLdb

#connection to database
#Settings for database connection
hostname = 'hostname'
username = 'username'
password = 'password'
database = 'databasename'

#Routine to insert tempearature records into weather3.Sensor table:
def insert_record(temp1, hum1, temp2, hum2):
	try:
		conn = MySQLdb.connect(host=hostname, user=username, passwd=password, db=database)
		cursor = conn.cursor()
		cursor.execute("INSERT INTO Sensor (temp, hum, temp2, hum2) VALUES (%s, %s, %s, %s)", [temp1, hum1, temp2, hum2])
		conn.commit()

	except Exception as error:
		print(error)

	finally:
		cursor.close()
		conn.close()


port = '/dev/ttyACM0'
baudrate = 115200
ser = serial.Serial(port,baudrate,timeout=1)

while True:
	data = ser.read(0)	#0 reading start position

	data = ser.readline()

#	print(data)

	if len(data) > 1:
		if data[0] == 'T' and data[1] == '1':
			if not data[4].isdigit():
				temp1 = ''.join(data[5:6])
			else:
				temp1 = ''.join(data[4:6])

			if not data[11].isdigit():
				hum1 = ''.join(data[12:13])
			else:
				hum1 = ''.join(data[11:13])


			if not data[19].isdigit():
				temp2 = ''.join(data[20:21])
			else:
				temp2 = ''.join(data[19:21])

			if not data[26].isdigit():
				hum2 = ''.join(data[27:28])
			else:
				hum2 = ''.join(data[26:28])


		#making time stamp
			dateStamp = datetime.datetime.now().strftime("%d-%m-%Y")
			dateStamp = str(dateStamp)

			timeStamp = datetime.datetime.now().strftime("%H:%M:%S")
			timeStamp = str(timeStamp)

		#open the file in the write mode
			f = open('/var/www/html/data.csv', 'a')

		# create the csv writer
			writer = csv.writer(f)

		# write a row to the csv file
			writer.writerow([dateStamp, timeStamp, str(temp1), str(hum1), str(temp2), str(hum2)])

		# close the file
			f.close()


		#sending data to database
			insert_record(str(temp1), str(hum1), str(temp2), str(hum2))

			sys.stdout.write("temp1: " + str(temp1) + '  ' + "hum1: " + str(hum1) + '  ' + "temp2: " + str(temp2) + '  ' + "hum2: " + str(hum2) + '\n')
			sys.stdout.flush()

