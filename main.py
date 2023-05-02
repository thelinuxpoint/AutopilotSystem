from src.vehicle import Vehicle
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import requests
import json

conn = "/dev/ttyACM0"
res = requests.post('https://dronedelivery.cu.ma/api/order-fetch')

if r.status_code != 200:
	exit()
else:
	print("APS|> GOT ORDER ")

response = json.loads(res.text)

latitude = float(response['success'][0]['lat'])
longitude = float(response['success'][0]['long'])

v = Vehicle(conn)

