import cv2 as cv
import numpy as np
import time
from PIL import Image
import dxcam
from pytesseract import pytesseract
import serial
from time import sleep



""" globale Informationen festlegen"""

arduino = serial.Serial('COM5', 115200 , write_timeout= 1) #Port ändern
sleep(3)
path_to_tesseract = r'C:\Users\Fuzzy-Projekt\AppData\Local\Tesseract-OCR\tesseract.exe'
pytesseract.tesseract_cmd = path_to_tesseract



"""globale Variabeln"""

i=0
Geschwindigkeit =""
HoechstePixelWeiß1x=0
HoechstePixelWeiß2x=0
HoechstePixelWeiß1y=0
HoechstePixelWeiß2y=0
Sonderfalllinks=0
Sonderfallrechts=0



"""Kamera inizialisierung"""

camera = dxcam.create()
camera.start()



"""Endlosschleife zum Analysieren der Frames"""

while True:

    #Startet die Zeitmessung
    start = time.time()
    


    """Screenshot des Bildschirmes"""

    img = camera.get_latest_frame()
    frame = np.array(img)


    
    """Maske zur Farbfilterung der Fahrbahnmakierungen"""

    untereGrenze=(0,0,0)
    obereGrenze=(200,200,200)
    Maske= cv.inRange(img, untereGrenze, obereGrenze)
      

     
    """Formatumwandlung"""

    im = Image.fromarray(Maske)
    width = im.size[0]
    height = im.size[1]

    """Begrenzung der Zählhöhe"""

    maximalehoehe= 385



    """Suchen der Fahrbahnmarkierung von der linken Seite des Bildes"""

    y = height
    x=20
    while True:
        y=y-1                                                       
        pixelzaehlhoehe= (x,y)                                      
        rgbzaehlhöhe=im.getpixel(pixelzaehlhoehe)                       #Auslesen des Grauwert eines Pixels
        
        
        if rgbzaehlhöhe==255 and pixelzaehlhoehe[1]==maximalehoehe:     #Wenn kein Fahrbahnrand gefunden und die maximale Zählhöhe erreicht wird die Schleife verlassen
            HoechstePixelWeiß1=pixelzaehlhoehe
            HoechstePixelWeiß1x=HoechstePixelWeiß1[0]
            break
        if y== maximalehoehe:                                           #Bei erreichen der Maximalhöhe werden die y Koordinaten wieder zurückgesetzt und die x Koordinaten um 50 erhöht
             y=720
             x=x+50
        if rgbzaehlhöhe==0:                                             #Bei erreichen der Fahrbahnmarkierung werden die y Koordinaten wieder zurückgesetzt und die x Koordinaten um 50 erhöht
             y=720
             x=x+50
        if x==1270:                                                     #Bei erreichen der x Koordinate von 1270 wird die Schleife verlassen
             HoechstePixelWeiß1x=20
             Sonderfalllinks=1
             break

    

    """Suchen der Fahrbahnmarkierung von der rechten Seite des Bildes"""

    y = height
    x=1260
    while True:
        y=y-1
        pixelzaehlhoehe= (x,y)
        rgbzaehlhöhe=im.getpixel(pixelzaehlhoehe)                       #Auslesen des Grauwert eines Pixels

        
        if rgbzaehlhöhe==255 and pixelzaehlhoehe[1]==maximalehoehe:     #Wenn kein Fahrbahnrand gefunden und die maximale Zählhöhe erreicht wird die Schleife verlassen
            HoechstePixelWeiß2=pixelzaehlhoehe
            HoechstePixelWeiß2x=HoechstePixelWeiß2[0]
            break
        if y== maximalehoehe:                                           #Bei erreichen der Maximalhöhe werden die y Koordinaten wieder zurückgesetzt und die x Koordinaten um 50 reduziert
             y=720
             x=x-50
        if rgbzaehlhöhe==0:                                             #Bei erreichen der Fahrbahnmarkierung werden die y Koordinaten wieder zurückgesetzt und die x Koordinaten um 50 reduziert
             y=720
             x=x-50 
        if x == 10:
            HoechstePixelWeiß2x=1260                                    #Bei erreichen der x Koordinate von 10 wird die Schleife verlassen
            Sonderfallrechts=1
            break

    
    if Sonderfalllinks==1 and Sonderfallrechts==1:
        x=20
        y=720
        while True:
            y=y-1
            pixelzaehlhoehe= (x,y)
            rgbzaehlhöhe=im.getpixel(pixelzaehlhoehe)

            if rgbzaehlhöhe==0:     
                HoechstePixelWeiß2=pixelzaehlhoehe
                HoechstePixelWeiß2y=HoechstePixelWeiß2[1]
                break
        
        x=1260
        y=720
        while True:
            y=y-1
            pixelzaehlhoehe= (x,y)
            rgbzaehlhöhe=im.getpixel(pixelzaehlhoehe)

            if rgbzaehlhöhe==0:     
                HoechstePixelWeiß2=pixelzaehlhoehe
                HoechstePixelWeiß2y=HoechstePixelWeiß2[1]
                break
        
        if HoechstePixelWeiß1y<HoechstePixelWeiß2y:
            HoechstePixelWeiß1x=1240
            HoechstePixelWeiß2x=1260
        
        if HoechstePixelWeiß1y>HoechstePixelWeiß2y:
            HoechstePixelWeiß1x=20
            HoechstePixelWeiß2x=40
        
        

    Sonderfalllinks=0
    Sonderfallrechts=0
        


    """Berechnung des Orientierungspunkts"""

    Abstand = int((HoechstePixelWeiß2x-HoechstePixelWeiß1x)/2)
    MittederStrecke=HoechstePixelWeiß1x+Abstand
    Pos=MittederStrecke-640
    Pos_Betrag=abs(Pos)
    

    
    """Geschwindigkeit ermitten dafür kann die 
    Häufigkeit in der if-Anweisung eingestellt werden"""

    i= i+1  
    
    if i == 5:     
        cut = Maske[20:110,30:190]                                  #von Frame zu Maske geändert
        Geschwindigkeit = pytesseract.image_to_string(cut)
        Geschwindigkeit = Geschwindigkeit [0:3]
        i=0
        
        

    """String Format anpassen"""
    string_Pos_Betrag=str(Pos_Betrag)
    #Konstante Länge des Strings festlegen
    if Pos_Betrag<100:
        if Pos_Betrag<10:
            string_Pos_Betrag = "00"+string_Pos_Betrag
        else:
            string_Pos_Betrag = "0"+string_Pos_Betrag
    #Vorzeichen Bit
    if Pos<0:
         string_Pos_Betrag= "0"+string_Pos_Betrag
    else:
        string_Pos_Betrag= "1"+string_Pos_Betrag
    
    
    
    
    """String zum senden vorbereiten"""
    string_to_send = Geschwindigkeit +  string_Pos_Betrag
    print(string_to_send)
    arduino.write(string_to_send.encode())
    


    """statistische Auswertung  || für Regelung nicht notwendig"""
    ende = time.time()
    DauerfuerDurchlauf= ende-start
    stringDauerfuerDurchlauf= str(DauerfuerDurchlauf)

    
    

    """Abbruchkriterien & Programm Beendigung"""
    
    if cv.waitKey(40) & 0xFF==ord("p"):
        arduino.close()        
        camera.stop() 
        break

        
  
    
arduino.close()        
camera.stop()   
cv.waitKey(0)
cv.destroyAllWindows()
quit()

