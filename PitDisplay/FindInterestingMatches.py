import requests
import gspread
from oauth2client.service_account import ServiceAccountCredentials
import time
import statbotics
import math

w = open("BlueAllianceAPI.txt", 'r')
Authkey = w.read()
w.close()

eventkey = "2026ncwak"

print("Getting all matches")
apiURL = "https://www.thebluealliance.com/api/v3/event/" + eventkey + "/matches"
resp = requests.get(apiURL, headers={'X-TBA-Auth-Key': Authkey})
data = resp.json()

#print(data)

#exit()

# Define the scope
scope = [
    'https://www.googleapis.com/auth/spreadsheets',
    'https://www.googleapis.com/auth/drive'
]

# Authenticate with credentials
credentials = ServiceAccountCredentials.from_json_keyfile_name('creds2.json', scope)
client = gspread.authorize(credentials)

# Open the Google Sheet
sheet = client.open_by_url('https://docs.google.com/spreadsheets/d/14U0JZ4Te7u0Prj2kdQF14Q0PRz7UKFYLrVq7CF0W7hI/edit?gid=1531398796#gid=1531398796')

worksheet = sheet.worksheet("Interesting Matches")

cells = worksheet.col_values(1)

inputCol = len(cells) + 1

# loop through all matches we are given
for i in range(0,len(data)):
    try:
        if data[i]['score_breakdown']["blue"]['totalAutoPoints'] >= 40:
            print("Found High Scoring Blue Auto: " + data[i]['comp_level'] + " " + str(data[i]['match_number']))
            if len(data[i]['videos']) != 0:
                if data[i]['videos'][0]['type'] == 'youtube':
                    video = "https://www.youtube.com/watch?v=" + data[i]['videos'][0]['key']
                else:
                    video = ""
            else:
                video = ""
            worksheet.insert_row([data[i]['event_key'], data[i]['match_number'], data[i]['key'], "Auto score blue", data[i]['score_breakdown']["blue"]['totalAutoPoints'], video], inputCol)
    except Exception as e:
        print(str(e))
        pass
    try:
        if data[i]['score_breakdown']["red"]['totalAutoPoints'] >= 40:
            print("Found High Scoring Red Auto: " + data[i]['comp_level'] + " " + str(data[i]['match_number']))
            if len(data[i]['videos']) != 0:
                if data[i]['videos'][0]['type'] == 'youtube':
                    video = "https://www.youtube.com/watch?v=" + data[i]['videos'][0]['key']
                else:
                    video = ""
            else:
                video = ""
            worksheet.insert_row([data[i]['event_key'], data[i]['match_number'], data[i]['key'], "Auto score red", data[i]['score_breakdown']["red"]['totalAutoPoints'], video], inputCol)
    except Exception as e:
        print(str(e))
        pass
    try:
        if data[i]['score_breakdown']["blue"]['totalPoints'] >= 125:
            print("Found High Scoring Blue Match: " + data[i]['comp_level'] + " " + str(data[i]['match_number']))
            if len(data[i]['videos']) != 0:
                if data[i]['videos'][0]['type'] == 'youtube':
                    video = "https://www.youtube.com/watch?v=" + data[i]['videos'][0]['key']
                else:
                    video = ""
            else:
                video = ""
            worksheet.insert_row([data[i]['event_key'], data[i]['match_number'], data[i]['key'], "Total score blue", data[i]['score_breakdown']["blue"]['totalPoints'], video], inputCol)
    except Exception as e:
        print(str(e))
        pass
    try:
        if data[i]['score_breakdown']["red"]['totalPoints'] >= 125:
            print("Found High Scoring Red Match: " + data[i]['comp_level'] + " " + str(data[i]['match_number']))
            if len(data[i]['videos']) != 0:
                if data[i]['videos'][0]['type'] == 'youtube':
                    video = "https://www.youtube.com/watch?v=" + data[i]['videos'][0]['key']
                else:
                    video = ""
            else:
                video = ""
            worksheet.insert_row([data[i]['event_key'], data[i]['match_number'], data[i]['key'], "Total score red", data[i]['score_breakdown']["red"]['totalPoints'], video], inputCol)
    except Exception as e:
        print(str(e))
        pass

