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

matchkey = []
teamnumbers = []
totalBoard = []
penaltyBoard = []
match_number = []
typematch = []


print("Getting all matches")
apiURL = "https://www.thebluealliance.com/api/v3/event/" + eventkey + "/matches"
resp = requests.get(apiURL, headers={'X-TBA-Auth-Key': Authkey})
data = resp.json()

#print(data)

#exit()

print("Got recieved formatting")

# loop through all matches we are given
for i in range(0,len(data)):
    if data[i]['winning_alliance'] == "":
        try:
            print("Match: " + str(data[i]['match_number']) + " has not finished yet")
        except:
            print("Bad match data :(")
        continue

    # loop through all blue alliances we are given
    for j in range(0,len(data[i]['alliances']['blue']['team_keys'])):

        # fill in team number, matchkey, board-score, alliance penalty, match number, and type
        teamnumbers.append(int(data[i]['alliances']['blue']['team_keys'][j].replace("frc","").strip()))
        matchkey.append(data[i]['key'])
        try:
            totalBoard.append(data[i]['alliances']['blue']['score'])
        except:
            totalBoard.append("")
        try:
            penaltyBoard.append(data[i]['score_breakdown']['blue']['foulPoints'])
        except:
            penaltyBoard.append("")
        try:
            match_number.append(data[i]['match_number'])
        except:
            match_number.append("")
        if data[i]['comp_level'] == "f":
            typematch.append("Final")
        elif data[i]['comp_level'] == "qm":
            typematch.append("Qualifer")
        elif data[i]['comp_level'] == "sf":
            typematch.append("Semi-Final")
        else:
            typematch.append("")

    # loop through all red alliances we are given
    for j in range(0,len(data[i]['alliances']['red']['team_keys'])):

        # fill in team number, matchkey, board-score, alliance penalty, match number, and type
        teamnumbers.append(int(data[i]['alliances']['red']['team_keys'][j].replace("frc","").strip()))
        matchkey.append(data[i]['key'])
        try:
            totalBoard.append(data[i]['alliances']['red']['score'])
        except:
            totalBoard.append("")
        try:
            penaltyBoard.append(data[i]['score_breakdown']['red']['foulPoints'])
        except:
            penaltyBoard.append("")
        try:
            match_number.append(data[i]['match_number'])
        except:
            match_number.append("")
        if data[i]['comp_level'] == "f":
            typematch.append("Final")
        elif data[i]['comp_level'] == "qm":
            typematch.append("Qualifer")
        elif data[i]['comp_level'] == "sf":
            typematch.append("Semi-Final")
        else:
            typematch.append("")

print("Formatted: " + str(len(data)) + " matches, and am about to create: " + str(len(teamnumbers)) + " entries")

#print(teamnumbers)
#print(matchkey)

# Define the scope
scope = [
    'https://www.googleapis.com/auth/spreadsheets',
    'https://www.googleapis.com/auth/drive'
]

# Authenticate with credentials
credentials = ServiceAccountCredentials.from_json_keyfile_name('credentials.json', scope)
client = gspread.authorize(credentials)

# Open the Google Sheet
sheet = client.open_by_url('https://docs.google.com/spreadsheets/d/1-AsEmhEqbABNO2hv2vS37AGeSQiSQW0leekwXtnfDy8/edit?resourcekey=&gid=780377629#gid=780377629')
#sheet = client.open_by_url('https://docs.google.com/spreadsheets/d/14U0JZ4Te7u0Prj2kdQF14Q0PRz7UKFYLrVq7CF0W7hI/edit?gid=1531398796#gid=1531398796')

worksheet = sheet.worksheet("Raw Data")

cells = worksheet.col_values(1)

inputCol = len(cells) + 1

print("Connected, Connecting to Statbotics")

sb = statbotics.Statbotics()

print("Connected, starting data feed")

# loop through all team numbers we have
for i in range(0,len(teamnumbers)):

    try:
        # try to get further match data
        print("Match Data sent for team: " + str(teamnumbers[i]) + " and match: " + str(matchkey[i]))
        data = sb.get_team_match(teamnumbers[i],matchkey[i])
        print("Match Data recieved from Statbotics")
    except Exception as e:
        print("Failed to get data. Error is: " + str(e))
        continue

    # get match breakdown
    breakdown = data['epa']['breakdown']
    color = data['alliance']

    #print(data)

    if breakdown['auto_tower'] > 0:
        ClimerAutoStatus = 1
        ClimberAuto = "Yes, successfully"
    else:
        ClimerAutoStatus = 0
        ClimberAuto = ""

    if breakdown['endgame_tower'] >= 30:
        EndgameL1 = 0
        EndgameL2 = 0
        EndgameL3 = 1
    elif breakdown['endgame_tower'] >= 20:
        EndgameL1 = 0
        EndgameL2 = 1
        EndgameL3 = 0
    elif breakdown['endgame_tower'] >= 10:
        EndgameL1 = 1
        EndgameL2 = 0
        EndgameL3 = 0
    else:
        EndgameL1 = 0
        EndgameL2 = 0
        EndgameL3 = 0

    if breakdown['endgame_fuel'] > 0:
        EndgameStatus = "Continue Shooting"
    else:
        EndgameStatus = ""

    #print(breakdown)

    output = [
        "", # Timestamp
        "Beep Boop 9000", # Your (REAL) Name
        teamnumbers[i], # Enter assigned Robot's Team Number
        typematch[i], # Match Type
        match_number[i], # Match Number
        "", # Did not compete: Click the button below if the robot did not make it onto the field. Otherwise, just click "Next".
        "", # Competition
        "", # Unique Match Id
        "", # Unique Match ID 2.0
        color, # Which color alliance is your robot on?
        ClimberAuto, # Does your robot climb to L1?
        math.floor(breakdown['auto_fuel']), # How many shots did your robot attempt during AUTO?
        "", # Does your robot refuel during AUTO?
        "", # How many cycles does your robot go through (how many times did it return to shoot fuel)?
        "", # Where can your robot pick up fuel cells from?	What does your robot do during endgame?
        "", # What does your robot do during endgame?
        "",
        "", # How many cycles (if any) did your robot complete during Endgame?
        totalBoard[i], # What was the TOTAL SCORE of your alliance as seen on the board?
        penaltyBoard[i], # How many penalty points did your alliance collect?
        "", # Was this robot disabled at any point during teleop/endgame?
        "", # Identify the type of robot you're scouting.
        "", # Did your robot play defense?
        math.floor(breakdown['auto_tower']), # Auto Hang Points
        math.floor(breakdown['auto_points']), # Auto Total Score
        math.floor(breakdown['teleop_points']), # Teleop Points
        math.floor(breakdown['endgame_tower']), # Endgame Hang Points
        math.floor(breakdown['endgame_points']), # Endgame Total
        "", # TOTAL	Disablement
        "", # Nothing
        "", # Unique Match ID
        ClimerAutoStatus, # Auto L1
        EndgameL3, # Endgame L3
        EndgameL2, # Endgame L2
        EndgameL1, # Endgame L1
        "", # Defense?
        math.floor(breakdown['total_points']) # Robot Total Score
        ]

    for h in range(0,len(output)):
        try:
            if output[h] < 0:
                output[h] = 0
        except:
            pass

    print("Sending Match Data To Google Sheets")
    worksheet.insert_row(output, inputCol)
    #exit()

    print("Waiting abit")
    print(str(100*((i+1)/len(teamnumbers)))[:4] + "% Done | " + str(i+1) + "/" + str(len(teamnumbers)))
    time.sleep(1)

