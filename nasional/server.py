import requests

ip_server = '192.168.10.244'

def getDummy1():
    dummy1 = requests.post("http://"+ip_server+"/robot/getdummy1.php")
    return dummy1.text.strip()

def getDummy2():
    dummy2 = requests.post("http://"+ip_server+"/robot/getdummy2.php")
    return dummy2.text.strip()

def getKiper():
    kiper = requests.post("http://"+ip_server+"/robot/getkiper.php")
    return kiper.text.strip()

def setStatus(id, status):
    requests.post("http://"+ip_server+"/robot/setstatus.php?"+"id="+str(id)+"&status="+str(status))

def getStatus(id):
    get = requests.post("http://"+ip_server+"/robot/getstatus.php?"+"id="+str(id))
    return get.text.strip()

def setGame(status):
    requests.post("http://"+ip_server+"/robot/setgame.php?"+"status="+str(status))

def getGame():
    game = requests.post("http://"+ip_server+"/robot/getgame.php")
    return game.text.strip()

def getMode():
    mode = requests.post("http://"+ip_server+"/robot/getmode.php")
    return mode.text.strip()

def getGameInfo():
    dummy1 = requests.post("http://"+ip_server+"/robot/getdummy1.php")
    dummy2 = requests.post("http://"+ip_server+"/robot/getdummy2.php")
    kiper = requests.post("http://"+ip_server+"/robot/getkiper.php")
    mode = requests.post("http://"+ip_server+"/robot/getmode.php")
    gameStatus = requests.post("http://"+ip_server+"/robot/getgame.php")
    return dummy1.text.strip(), dummy2.text.strip(), kiper.text.strip(), mode.text.strip(), gameStatus.text.strip()