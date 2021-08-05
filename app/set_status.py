import requests

# webserver
ip_server = "192.168.10.244"
def setStatus(id, status):
    requests.post("http://"+ip_server+"/robot/setstatus.php?"+"id="+str(id)+"&status="+str(status))
def getStatus(id):
    get = requests.post("http://"+ip_server+"/robot/getstatus.php?"+"id="+str(id))
    return get.text.strip()

id = input("Input ID: ")

while True:
    setStat = input("Set Status: ")
    setStatus(id, setStat)
    print (getStatus(id))
