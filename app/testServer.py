import requests

ip_server = "192.168.10.244"
def setStatus(id, status):
    requests.post("http://"+ip_server+"/robot/setstatus.php?"+"id="+str(id)+"&status="+str(status))
def getStatus(id):
    get = requests.post("http://"+ip_server+"/robot/getstatus.php?"+"id="+str(id))
    return get.text.strip()
def getGameInfo():
    dummy1 = requests.post("http://"+ip_server+"/robot/getdummy1.php")
    dummy2 = requests.post("http://"+ip_server+"/robot/getdummy2.php")
    kiper = requests.post("http://"+ip_server+"/robot/getkiper.php")
    mode = requests.post("http://"+ip_server+"/robot/getmode.php")
    gameStatus = requests.post("http://"+ip_server+"/robot/getgame.php")
    return dummy1.text.strip(), dummy2.text.strip(), kiper.text.strip(), mode.text.strip(), gameStatus.text.strip()

d1, d2, k, m, s = getGameInfo()
print(d1,d2,k,m,s)