import gspread
from oauth2client.service_account import ServiceAccountCredentials
import requests
import turtle
import time
import math
import random

global turt, sizeX, sizeY, lastmatchkey, NxtBluTeamKeys, NxtRedTeamKeys

turt = turtle.Turtle()
turtle.tracer(0,0)
turtle.bgcolor("cornsilk4")

# TODO: put into loop
sizeX = turt.getscreen().xscale
sizeY = turt.getscreen().yscale

def askBlueAlliance():
    global lastmatchkey, NxtBluTeamKeys, NxtRedTeamKeys

    teamkey = "frc5607"

    # <><><><><><> CHANGE THIS <><><><><><>
    eventkey = "2026week0"
    # <><><><><><> CHANGE THIS <><><><><><>

    AuthkeyFile = open("BlueAllianceAPI.txt")
    Authkey = AuthkeyFile.read()
    AuthkeyFile.close()

    apiURL = "https://www.thebluealliance.com/api/v3/team/" + teamkey + "/event/" + eventkey + "/status"
    resp = requests.get(apiURL, headers={'X-TBA-Auth-Key': Authkey})
    data = resp.json()

    if data['next_match_key'] is not None:
        if lastmatchkey is not None:
            if lastmatchkey is data['next_match_key']:
                lstmatch = "https://www.thebluealliance.com/api/v3/match/" + data['next_match_key']
                nxtmatchresp = requests.get(lstmatch, headers={'X-TBA-Auth-Key': Authkey})
                nxtmatchresp = nxtmatchresp.json()

                NxtName = nxtmatchresp['key']
                NxtTime = nxtmatchresp['time']
                NxtAliData = nxtmatchresp['alliances']
                NxtBlueAliData = NxtAliData['blue']
                NxtBluTeamKeys = NxtBlueAliData['team_keys']
                NxtRedAliData = NxtAliData['red']
                NxtRedTeamKeys = NxtRedAliData['team_keys']
                return [NxtBluTeamKeys, NxtRedTeamKeys, [NxtName, NxtTime], True]

        lastmatchkey = data['next_match_key']
    return [None, None, [None, None], False]

def askBlueAllianceMatch(MatchKey):
    global lastmatchkey, NxtBluTeamKeys, NxtRedTeamKeys

    AuthkeyFile = open("BlueAllianceAPI.txt")
    Authkey = AuthkeyFile.read()
    AuthkeyFile.close()

    lstmatch = "https://www.thebluealliance.com/api/v3/match/" + MatchKey
    nxtmatchresp = requests.get(lstmatch, headers={'X-TBA-Auth-Key': Authkey})
    nxtmatchresp = nxtmatchresp.json()
    print(nxtmatchresp)

    NxtName = nxtmatchresp['key']
    NxtAliData = nxtmatchresp['alliances']
    NxtBlueAliData = NxtAliData['blue']
    NxtBluTeamKeys = NxtBlueAliData['team_keys']
    NxtRedAliData = NxtAliData['red']
    NxtRedTeamKeys = NxtRedAliData['team_keys']
    return [NxtBluTeamKeys, NxtRedTeamKeys, NxtName, True]

def drawField(counter):
    global turt, sizeX, sizeY
    turt.clear()
    turt.home()

    turt.fillcolor("Gray")
    #turt.right(90)
    turt.forward(720*sizeX)
    turt.left(90)
    turt.forward(410*sizeY)
    turt.right(180)
    turt.begin_fill()
    turt.down()
    for i in range(0,2):
        turt.forward(810*sizeY)
        turt.right(90)
        turt.forward(1440*sizeX)
        turt.right(90)
    turt.up()
    turt.end_fill()

    turt.fillcolor("DeepSkyBlue")
    turt.begin_fill()
    turt.down()
    for i in range(0,2):
        turt.forward(810*sizeY)
        turt.right(90)
        turt.forward(220*sizeX)
        turt.right(90)
    turt.up()
    turt.end_fill()

    turt.forward(810*sizeY)
    turt.right(90)
    turt.forward(1440*sizeX)
    turt.right(90)
    turt.fillcolor("firebrick1")
    turt.begin_fill()
    turt.down()
    for i in range(0,2):
        turt.forward(810*sizeY)
        turt.right(90)
        turt.forward(220*sizeX)
        turt.right(90)
    turt.up()
    turt.end_fill()

    turt.home()
    turt.backward(720)
    turt.left(90)
    turt.forward(450)
    turt.right(90)
    turt.color("Blue")
    turt.pensize(15)
    turt.down()
    turt.forward(1440*(counter/30))
    turt.up()
    turt.color("Black")
    turt.pensize(1)
    turt.home()
    


def populateData(blue, red, fieldData):
    global turt, sizeX, sizeY

    if(len(blue) < 3):
        print("Not enough blue Data")
        turt.write("Not enough data", align="center", font=('Arial', 40, 'normal'))
        #return
    elif(len(red) < 3):
        print("Not enough red Data")
        turt.write("Not enough data", align="center", font=('Arial', 40, 'normal'))
        #return

    turt.forward(720*sizeX)
    turt.left(90)
    turt.forward(410*sizeY)
    turt.right(180)
    turt.forward(100*sizeY)
    turt.right(90)
    turt.forward(200*sizeX)
    turt.left(90)
    if len(blue) >= 1:
        turt.write(blue[0][0], font=('Arial', 48, 'normal'))
        turt.forward(100*sizeY)
        turt.write(blue[0][1] + " - " + blue[0][2], font=('Arial', 30, 'normal'))

    if len(blue) >= 2:
        turt.forward(205*sizeY)
        turt.write(blue[1][0], font=('Arial', 48, 'normal'))
        turt.forward(100*sizeY)
        turt.write(blue[1][1] + " - " + blue[1][2], font=('Arial', 30, 'normal'))

    if len(blue) >= 3:
        turt.forward(205*sizeY)
        turt.write(blue[2][0], font=('Arial', 48, 'normal'))
        turt.forward(100*sizeY)
        turt.write(blue[2][1] + " - " + blue[1][2], font=('Arial', 30, 'normal'))

    turt.home()

    turt.backward(680*sizeX)
    turt.left(90)
    turt.forward(410*sizeY)
    turt.right(180)
    turt.forward(100*sizeY)
    turt.right(90)
    #turt.forward(200*sizeX)
    turt.left(90)
    if len(red) >= 1:
        turt.write(red[0][0], font=('Arial', 48, 'normal'))
        turt.forward(100*sizeY)
        turt.write(red[0][1] + " - " + red[0][2], font=('Arial', 30, 'normal'))

    if len(red) >= 2:
        turt.forward(205*sizeY)
        turt.write(red[1][0], font=('Arial', 48, 'normal'))
        turt.forward(100*sizeY)
        turt.write(red[1][1] + " - " + red[1][2], font=('Arial', 30, 'normal'))

    if len(red) >= 3:
        turt.forward(205*sizeY)
        turt.write(red[2][0], font=('Arial', 48, 'normal'))
        turt.forward(100*sizeY)
        turt.write(red[2][1] + " - " + red[1][2], font=('Arial', 30, 'normal'))

    turt.home()
    if fieldData is not None:
        turt.left(90)
        turt.forward(303)
        turt.write(fieldData, align="center",font=('Arial', 30, 'bold'))

    # titles to be assigned:
    # - Highest Shield > Best defense
    # - Auto Hang > Hangs In Auto
    # - Highest Endgame > Best Endgame
    # - Highest Teleop > Best Teleop
    # - High Disablement > Disabled Often
    # - Highest Total points > MVP
    # - Total is 0 > Has Not Scored

    turt.home()

    highestShieldBlue = 0
    highestShieldRed = 0
    highestShieldIndexB = 0
    highestShieldIndexR = 0

    highestoverallShield = 0

    highestEndgameBlue = 0
    highestEndgameRed = 0
    highestEndgameIndexB = 0
    highestEndgameIndexR = 0

    highestoverallEndgame = 0

    highestTotalBlue = 0
    highestTotalRed = 0
    highestTotalIndexB = 0
    highestTotalIndexR = 0

    highestoverallTotal = 0

    highestTeleopBlue = 0
    highestTeleopRed = 0
    highestTeleopIndexB = 0
    highestTeleopIndexR = 0

    highestoverallTeleop = 0

    for i in range(0,len(blue)):
        try:
            if int(blue[i][1].replace("#", "").strip()) > highestShieldBlue:
                highestShieldBlue = int(blue[i][1].replace("#", "").strip())
                highestShieldIndexB = i
        except Exception as e:
            pass
        try:
            if int(blue[i][3].replace("#", "").strip()) > highestEndgameBlue:
                highestEndgameBlue = int(blue[i][3].replace("#", "").strip())
                highestEndgameIndexB = i
        except Exception as e:
            pass
        try:
            if int(blue[i][7].replace("#", "").strip()) > highestTotalBlue:
                highestTotalBlue = int(blue[i][7].replace("#", "").strip())
                highestTotalIndexB = i
        except Exception as e:
            pass
        try:
            if int(blue[i][4].replace("#", "").strip()) > highestTeleopBlue:
                highestTeleopBlue = int(blue[i][4].replace("#", "").strip())
                highestTeleopIndexB = i
        except Exception as e:
            pass

    for i in range(0,len(red)):
        try:
            if int(red[i][1].replace("#", "").strip()) > highestShieldRed:
                highestShieldRed = int(red[i][1].replace("#", "").strip())
                highestShieldIndexR = i
        except Exception as e:
            pass
        try:
            if int(red[i][3].replace("#", "").strip()) > highestEndgameRed:
                highestEndgameRed = int(red[i][3].replace("#", "").strip())
                highestEndgameIndexR = i
        except Exception as e:
            pass
        try:
            if int(red[i][7].replace("#", "").strip()) > highestTotalRed:
                highestTotalRed = int(red[i][7].replace("#", "").strip())
                highestTotalIndexR = i
        except Exception as e:
            pass
        try:
            if int(red[i][4].replace("#", "").strip()) > highestTeleopRed:
                highestTeleopRed = int(red[i][4].replace("#", "").strip())
                highestTeleopIndexR = i
        except Exception as e:
            pass

    if highestShieldBlue > highestShieldRed:
        highestoverallShield = blue[highestShieldIndexB][0]
    elif highestShieldBlue < highestShieldRed:
        highestoverallShield = red[highestShieldIndexR][0]


    if highestEndgameBlue > highestEndgameRed:
        highestoverallEndgame = blue[highestEndgameIndexB][0]
    elif highestEndgameBlue < highestEndgameRed:
        highestoverallEndgame = red[highestEndgameIndexR][0]


    if highestTotalBlue > highestTotalRed:
        highestoverallTotal = blue[highestTotalIndexB][0]
    elif highestTotalBlue < highestTotalRed:
        highestoverallTotal = red[highestTotalIndexR][0]


    if highestTeleopBlue > highestTeleopRed:
        highestoverallTeleop = blue[highestTeleopIndexB][0]
    elif highestTeleopBlue < highestTeleopRed:
        highestoverallTeleop = red[highestTeleopIndexR][0]

    #print("Best shield is: " + str(highestoverallShield))
    #print("Best Endgame is: " + str(highestoverallEndgame))
    #print("Best Teleop is: " + str(highestoverallTeleop))
    #print("Match MVP is: " + str(highestoverallTotal))

    turt.backward(480*sizeX)
    turt.left(90)
    turt.forward(410*sizeY)
    turt.right(180)
    turt.forward(100*sizeY)
    turt.right(90)
    #turt.forward(200*sizeX)
    turt.left(90)
    turt.forward(105*sizeY)
    for i in range(0,len(red)):
        if red[i][0] == str(highestoverallTotal):
            turt.write("MVP", font=('Arial', 20, 'normal'))
        elif red[i][0] == str(highestoverallTeleop):
            turt.write("Highest Teleop AVG", font=('Arial', 20, 'normal'))
        elif red[i][0] == str(highestoverallShield):
            turt.write("Best Defense", font=('Arial', 20, 'normal'))
        elif red[i][0] == str(highestoverallEndgame):
            turt.write("Highest Endgame AVG", font=('Arial', 20, 'normal'))
        elif int(red[i][6].replace("%","").strip()) >= 25:
            turt.write("Disabled: " + red[i][6], font=('Arial', 20, 'normal'))
        else:
            try:
                if int(red[i][8].replace("%","").strip()) == 0:
                    turt.write("Has Not Scored", font=('Arial', 20, 'normal'))
            except:
                pass
        turt.forward(290*sizeY)
    
    turt.home()
    turt.forward(425*sizeX)
    turt.left(90)
    turt.forward(410*sizeY)
    turt.right(180)
    turt.forward(100*sizeY)
    turt.right(90)
    turt.forward(200*sizeX)
    turt.left(90)
    turt.forward(105*sizeY)
    for i in range(0,len(blue)):
        if blue[i][0] == str(highestoverallTotal):
            turt.write("MVP", font=('Arial', 20, 'normal'))
        elif blue[i][0] == str(highestoverallTeleop):
            turt.write("Highest Teleop AVG", font=('Arial', 20, 'normal'))
        elif blue[i][0] == str(highestoverallShield):
            turt.write("Best Defense", font=('Arial', 20, 'normal'))
        elif blue[i][0] == str(highestoverallEndgame):
            turt.write("Highest Endgame AVG", font=('Arial', 20, 'normal'))
        elif int(blue[i][6].replace("%","").strip()) >= 25:
            turt.write("Disabled: " + blue[i][6], font=('Arial', 20, 'normal'))
        else:
            try:
                if int(blue[i][8].replace("%","").strip()) == 0:
                    turt.write("Has Not Scored", font=('Arial', 20, 'normal'))
            except:
                pass
        turt.forward(290*sizeY)
            
    
    turt.home()

    totalred = 0
    totalblue = 0
    for i in range(0, len(blue)):
        try:
            totalblue += int(blue[i][7].replace("#", "").strip())
        except Exception as e:
            pass
        try:
            totalred += int(red[i][7].replace("#", "").strip())
        except Exception as e:
            pass
    
    #print(str(totalblue) + " - " + str(totalred))

    if totalblue == totalred:
        turt.left(90)
        turt.forward(210*sizeY)
        turt.write("Expected Close Match", align="center", font=('Arial', 33, 'normal'))
        turt.home()

        turt.backward(480*sizeX)
        turt.left(90)
        turt.forward(410*sizeY)
        turt.right(180)
        turt.forward(100*sizeY)
        turt.right(90)
        #turt.forward(200*sizeX)
        turt.left(90)
        turt.forward(50*sizeY)
        turt.write("Score: " + str(totalblue), font=('Arial', 20, 'normal'))
        turt.home()

        turt.forward(570*sizeX)
        turt.left(90)
        turt.forward(410*sizeY)
        turt.right(180)
        turt.forward(100*sizeY)
        turt.right(90)
        turt.forward(200*sizeX)
        turt.left(90)
        turt.forward(50*sizeY)
        turt.write("Score: " + str(totalred), font=('Arial', 20, 'normal'))
        turt.home()

        turt.turtlesize(5,5,4)
        turt.fillcolor("Gray")
        turt.left(90)
        
    elif totalblue - 20 > totalred:
        turt.backward(480*sizeX)
        turt.left(90)
        turt.forward(435*sizeY)
        turt.right(180)
        turt.forward(100*sizeY)
        turt.right(90)
        #turt.forward(200*sizeX)
        turt.left(90)
        turt.write("Blu Expected To Win", font=('Arial', 30, 'normal'))
        turt.forward(50*sizeY)
        turt.write("Score: " + str(totalblue), font=('Arial', 20, 'normal'))
        turt.home()

        turt.forward(570*sizeX)
        turt.left(90)
        turt.forward(410*sizeY)
        turt.right(180)
        turt.forward(100*sizeY)
        turt.right(90)
        turt.forward(200*sizeX)
        turt.left(90)
        turt.forward(50*sizeY)
        turt.write("Score: " + str(totalred), font=('Arial', 20, 'normal'))
        turt.home()

        turt.turtlesize(5,5,4)
        turt.fillcolor("Blue")
        turt.left(180)
        
    elif totalred - 20 > totalblue:
        turt.forward(250*sizeX)
        turt.left(90)
        turt.forward(435*sizeY)
        turt.right(180)
        turt.forward(100*sizeY)
        turt.right(90)
        turt.forward(200*sizeX)
        turt.left(90)
        turt.write("Red Expected To Win", font=('Arial', 30, 'normal'))
        turt.forward(50*sizeY)
        turt.left(90)
        turt.forward(270*sizeX)
        turt.write("Score: " + str(totalred), font=('Arial', 20, 'normal'))
        turt.home()

        turt.backward(480*sizeX)
        turt.left(90)
        turt.forward(410*sizeY)
        turt.right(180)
        turt.forward(100*sizeY)
        turt.right(90)
        #turt.forward(200*sizeX)
        turt.left(90)
        turt.forward(50*sizeY)
        turt.write("Score: " + str(totalblue), font=('Arial', 20, 'normal'))
        turt.home()

        turt.turtlesize(5,5,4)
        turt.fillcolor("Red")
        

    else:
        turt.left(90)
        turt.forward(210*sizeY)
        turt.write("Expected Close Match", align="center", font=('Arial', 33, 'normal'))
        turt.home()

        turt.backward(480*sizeX)
        turt.left(90)
        turt.forward(410*sizeY)
        turt.right(180)
        turt.forward(100*sizeY)
        turt.right(90)
        #turt.forward(200*sizeX)
        turt.left(90)
        turt.forward(50*sizeY)
        turt.write("Score: " + str(totalblue), font=('Arial', 20, 'normal'))
        turt.home()

        turt.forward(570*sizeX)
        turt.left(90)
        turt.forward(410*sizeY)
        turt.right(180)
        turt.forward(100*sizeY)
        turt.right(90)
        turt.forward(200*sizeX)
        turt.left(90)
        turt.forward(50*sizeY)
        turt.write("Score: " + str(totalred), font=('Arial', 20, 'normal'))
        turt.home()

        turt.turtlesize(5,5,4)
        turt.fillcolor("Gray")
        turt.left(90)

    




def getScouting(blue, red):

    for i in range(0,len(blue)):
        blue[i] = blue[i].replace("frc","").strip()
    for i in range(0,len(red)):
        red[i] = red[i].replace("frc","").strip()

    newBlue = []
    newRed = []

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

    worksheet = sheet.worksheet("RAPTOR & SHIELD")

    # Fetch the first row of data
    #teamNums = worksheet.row_values(1)
    #print(f"Eval Data types: {teamNums}")
    #teamNums = worksheet.col_values(1)
    #print(f"Eval team numbers: {teamNums}")

    list_of_lists = worksheet.get_all_values()

    for j in range(0,len(blue)):
        for i in range(0,len(list_of_lists)): 
            if str(list_of_lists[i][0]) == blue[j]:
                print("=============================")
                print("Match found blu adding: ")
                print("Number: " + str(blue[j]))
                print("Raptor: " + str(list_of_lists[i][10])[:2])
                print("Shield: " + str(list_of_lists[i][11])[:2])
                print("Auto Hang: " + str(list_of_lists[i][1])[:2])
                print("Endgame: " + str(list_of_lists[i][2])[:2])
                print("Teleop: " + str(list_of_lists[i][3])[:2])
                print("Disablement: " + str(list_of_lists[i][5])[:2])
                print("Total: " + str(list_of_lists[i][8])[:2])

                newBlue.append([
                    blue[j], 
                    str(list_of_lists[i][10])[:2], # Raptor
                    str(list_of_lists[i][11])[:2], # Shield
                    str(list_of_lists[i][1])[:2], # Auto Hang
                    str(list_of_lists[i][2])[:2], # Endgame
                    str(list_of_lists[i][3])[:2], # Teleop
                    str(list_of_lists[i][5])[:2], # Disablement
                    str(list_of_lists[i][8])[:2], ]) # Total points
            if str(list_of_lists[i][0]) == red[j]:
                print("=============================")
                print("Match found red adding: ")
                print("Number: " + str(red[j]))
                print("Raptor: " + str(list_of_lists[i][10])[:2])
                print("Shield: " + str(list_of_lists[i][11])[:2])
                print("Auto Hang: " + str(list_of_lists[i][1])[:2])
                print("Endgame: " + str(list_of_lists[i][2])[:2])
                print("Teleop: " + str(list_of_lists[i][3])[:2])
                print("Disablement: " + str(list_of_lists[i][5])[:2])
                print("Total: " + str(list_of_lists[i][8])[:2])

                newRed.append([
                    red[j], 
                    str(list_of_lists[i][10])[:2], # Raptor
                    str(list_of_lists[i][11])[:2], # Shield
                    str(list_of_lists[i][1])[:3], # Auto Hang
                    str(list_of_lists[i][2])[:2], # Endgame
                    str(list_of_lists[i][3])[:2], # Teleop
                    str(list_of_lists[i][5])[:2], # Disablement
                    str(list_of_lists[i][8])[:2],]) # Total points

    print(newBlue)
    print(newRed)

    return [newBlue, newRed]

counter = 30

while True:
    if counter >= 30:
        counter = 0
        #[bluename, redname, fieldData, diff] = askBlueAlliance()
        #if bluename == None:
        #    print("CANNOT GET BLUE ALLIANCE DATA")
        #    exit()

        redname = []
        bluename = []
        #fieldData = ["Qualifer " + str(random.randrange(0,99)), "12:33 PM"]

        [bluename, redname, fieldData, diff] = askBlueAllianceMatch("2026week0_qm3")

        #Avalnames = ['1', '2', '3', '4', '5', '6', '123', '345', '456', '567', '1234']
        #for i in range(0,3):
        #    redname.append(Avalnames[random.randrange(0,len(Avalnames))])
        #    bluename.append(Avalnames[random.randrange(0,len(Avalnames))])


        if diff:
            [blue, red] = getScouting(bluename, redname)
    else:
        counter += 1


    drawField(counter)
    populateData(blue, red, fieldData)
    turtle.update()
    time.sleep(1)


# get blue alliance data
# if next match is different then
# - get scouting data
# - get who is on what alliance
# - clear screen
# - write blue alliance
# - write red alliance
# wait 1 minute

#for i in range(0,len(list_of_lists)):
#    if()

