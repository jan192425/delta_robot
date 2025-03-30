# Aufbau einer Compliant-Delta-Picker-Kinematik für eine intuitive Eingabe eines Mobilen Roboters

Nachfolgend ist die Projektarbeit von Jan Sopejstal, Thomas Tasler und Nick Müller beschrieben. Die Arbeit wurde im Wintersemester 24/25 an der technischen Hochschule Nürnberg durchgeführt und von Herrn Prof. Dr. rer. nat. Christian Pfitzner betreut.

## Inhaltsverzeichnis

- [1. Projektbeschreibung / Ziel des Projektes](#1-projektbeschreibung--ziel-des-projektes)
- [2. Komponenten des Delta-Pickers](#2-komponenten-des-delta-pickers)
- [3. Auslegung der Motoren](#3-auslegung-der-motoren)
- [4. Umsetzung der zwei Modis](#4-umsetzung-der-zwei-modis)
    - [4.1 Setup](#41-setup)
    - [4.2 Auswählbare Modi](#42-auswählbare-modi)
        - [4.2.1 Punktnavigation (mode: 0)](#421-punktnavigation-mode-0)
        - [4.2.2 Compliant-Modus (mode: 1)](#422-compliant-modus-mode-1)
- [5. Simulator für Delta-Roboter](#5-simulator-für-delta-roboter)
---

## 1. Projektbeschreibung / Ziel des Projektes
Für die Steuerung eines mobilen Robotes soll eine Delta-Kinematik aufgebaut werden. Durch eine geeignete Auswahl an Motoren und Ansteuerung soll die Kinematik einen vorgegebenen Zielpunkt anfahren können. Darüber hinaus, soll die Kinematik auch durch Interaktion in eine Richtung von Hand bewegt werden und bei eine einem Loslassen zurück in die Ursprungsposition fahren. Notwendig ist hierfür eine dynamische Regelung.

## 2. Komponenten des Delta-Pickers
Der Delta-Roboter besteht grundlegend aus einer [Grundplatte](/images/base.png), [drei Baugleichen Armen](/images/arm.png) sowie einer [Endeffektorplatte](/images/endeffektor.png). An der [Grundplatte](/images/base.png) werden zudem die Motorhalterungen sowie ein Gehäuse montiert. Das Gehäuse dient zur Befestigung von Bauteilen, welche für die Ansteuerung der Motoren erforderlich sind. Nachfolgend ist der Gesamtaufbau dargestellt.

![Delta_Roboter](/images/delta_robot.png)

Die nicht zugekauften Bauteile des Delta Roboters sind als [stl Datein](/stl/) abgelegt. In der folgenden Tabelle sind die für die Montage erfolgerlichen Komponenten aufgelistet. Darin enhalten sind sowohl die Zukaufkaufteile als auch die notwendige Anzahl der jeweiligen stl-Datei.

|**Bauteil - Zukauf**|**Stückzahl**|
|-----------------|:------------------:|
|Dynamixel XM540-W270-R|3|
|ROBOTIS DYNAMIXEL U2D2 Power Hub|1|
|ROBOTIS DYNAMIXEL U2D2|1|
|Gabelkopf DIN 71752 6x12|12|
|Bolzen für Gabelkopf ø6mm|12|
|Gewindestange M6x1x41.5|6|
|Zylinderschraube mit Innensechskant ISO 4762 M2,5 x 25|24|
|Senkschraube mit Innensechsrund ISO 14581 M3 x 16|15|
|Senkschraube mit Innensechsrund ISO 14581 M3 x 10|24|
|Sechskantmuttern DIN 934 M3|24|
|M3 Schmelzgewindeeinsatz|15|
|||
|**Bauteil - nicht Zukauf**|**Stückzahl**|
|[base housing](/stl/base_housing.stl)|1|
|[base plate](/stl/base_plate.stl)|1|
|[bicep rod](/stl/bicep_rod.stl)|3|
|[end effector base](/stl/end_effector_base.stl)|1|
|[forarm rod divided 1](/stl/forarm_rod_divided_1.stl)|6|
|[forarm rod divided 2](/stl/forarm_rod_divided_2.stl)|6|
|[motor mount](/stl/motor_mount.stl)|3|

**Kabel ergänzen**

## 3. Auslegung der Motoren

Für die Auswahl der Motoren ist das maximal auftretende Drehmoment und die maximale Drehzahl von Bedeutung. Da der Delta-Roboter eine geschlossene Kinematik besitzt ist die Berechnung insbesondere für den dynamischen Fall von hoher Komplexität. Aus diesem Grund wurde nur der statische Fall betrachtet und das Ergebnis mit einem Sicherheitsfaktor verrechnet.

Die Berechnung des maximal auftretenden Drehmoments erfolgt mithilfe eines [Matlab Skripts](/matlab/delta_picker_motorenauslegung.mlx). Dadurch ist es möglich, die Motormomente für jede Effektorposition im Arbeitsraum zu ermitteln. Für eine statische Last von etwa 4.0 kg auf den Endeffektor ergeben sich die folgenden Motormomente.

[![Motormomente](/images/motorenauslegung.png)](/images/motorenauslegung.png)

Das maximal erforderliche Motormoment errechnet sich somit zu 7.6 Nm. Der DYNAMIXEL XM540-W270-R kann ein Drehmoment von bis zu 12.9 Nm aufbringen. Des Weiteren verfügen DYNAMIXEL Motoren über eine integrierte Regelung und ein dazu passendes Software Development Kit. Aufgrund dieser Vorteile wurde der oben genannte DYNAMIXEL anstelle eines konventionellen Motors ausgewählt.

## 4. Robotersteuerung mit ROS2
Die Kommunikation zwischen Motoren und Computer erfolgt in diesem Projekt mittels ROS2.

### 4.1 Setup
Für die Ansteuerung des Delta-Roboters müssen folgende Schritte zur Vorbereitung gemacht werden:
1.  Download des DynamixelSDK-Repositorys:
```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```
2. Download des delta_robot-Repositorys 
```
git clone https://github.com/jan192425/delta_robot.git
```
3. Installieren der Packete
```
colcon build 
```
4. sourcen der setup.bash Datei
```
source install/setup.bash
```
5. Starten der Nodes in einzelnen Terminals 
```
ros2 run delta_robot opmod_node
```
```
ros2 run delta_robot commot_node
```
**Hinweis:** <br> Erst das Starten der COMMOT Node setzt den Torque_enable der Motoren auf 1 (Bestromung der Motoren). **=> Arme fallen herunter wenn COMMOT Node gestoppt wird!!**

### 4.2 Auswählbare Modi
Im Rahmen dieses Projektes soll der Delta-Roboter über zwei Modis verfügen. Erstens einen Modus zur Punktnavigation, d.h. der Anwender gibt eine Position des Endeffektors im Arbeitsraum vor. Durch Berechnung der inversen Delta-Kinematik können die erforderlichen Motorwinkel ermittelt und mitteles eines Regelkreises eingestellt werden. 

Des Weiteren soll ein Compliant-Modus implementiert werden. Der Endeffektor soll durch den Anwender ausgelenkt werden können und beim Loslassen wieder in den Ursprung zurückfahren. Wenn der Effektor zu einem gewissen Grad aus der Mitte gelenkt ist, soll diese Information an den Mobilen Roboter weitergeleitet werden. Dieser soll als Reaktion hierauf sich - und dadurch auch das Zentrum der Delta-Grundplatte- wieder unter den Endeffektor bewegen. 

Zwischen den zwei Modi kann mit dem Nachrichten-Typ *OpMod* hin-und hergeschaltet werden. Dieser Nachrichten-Typ ist selbst definiert und im package delta_robot_interfaces zu finden. Im Folgenden werden beide Modi und deren Struktur in ROS2 anhand von Bildern vorgestellt. 

#### 4.2.1 Punktnavigation (mode: 0)
Für die Aktivierung des Punktnavigationsmodus muss (nach Start der 2 Nodes) folgende Nachricht publiziert werden, wobei an den ... Stellen die gewünschte Endeffektorposition anzugeben ist:
- in neuem Terminal:
```
source install/setup.bash
```
```
ros2 topic pub -1 OpMod delta_robot_interfaces/OpMod "{mode: 0, xeff: ..., yeff: ..., zeff: ...}"
```
**Hinweis:** <br>
Die Ausrichtung des Koordinatensystems ist auf der Grundplatte des Delta-Roboters zu finden.

Für den Fall, dass der vorgegebene Punkt aufgrund der Geometrie und der daraus resultierenden Kinematik des Roboters nicht zu erreichen ist, verharrt der Delta-Roboter auf der letzten zulässigen Position. Der Roboter wird ebenfalls in der letzten Position verharren, wenn die gewünschte Effktorposition Motorwinkel <= -30° oder >=90° bedeuten würden. Dadurch wird im ersten Fall die Kollision mit der Grundplatte verhindert. Im zweiten Fall wird so die kinematische Singularität der Delta-Kinematik verhindert, die aus der bei 90° parallel zueinander stehenden  Bicepsgliedern resultieren würde.

Die untenstehende Abbildung zeigt den Datenfluss zwischen den ROS2 Nodes COMMOT und OPMOD, der ausgelagerten Kinematikberechnung und den Motoren. Dabei steht T für das Terminal bzw. die dort publizierte *OpMod* Nachricht. Weiterhin ist zu erkennen, dass sich die Regelparamter in COMMOT Node verändern lassen für den Fall, dass das aktuell implementierte Verhalten der Positionsregelung für die sepezifische Anwendung angepasst werden muss.


[![Punktnavi](/images/Pointnav.png)](/images/Pointnav.png)


#### 4.2.2 Compliant-Modus (mode: 1)
**Umsetzung des "compliant behaviours":** <br>
Die Realisierung eines Roboters mit nachgiebigen Verhaltens (engl. "compliant behaviour") bedarf der Herleitung des komplexen Dynamiksystems der closed-loop Delta-Kinematik und einer Regelung, die in den meisten Fällen eine Drehmomentüberwachung benötigt. Da Motoren mit integrierter Drehmomentüberwachung i.d.R. ein Vielfaches mehr kosten, sollte im Rahmen dieses Projekts untersucht werden, ob man ein ähnliches Verhalten auch mit Motoren ohne Drehmomentüberwachung und ohne die teils nur numerisch mögliche Modellrechnung realsieren kann.

Der Ansatz, der hierbei verfolgt wurde, sieht eine adaptive Änderung des P-Gains in Abhängigkeit der individuellen Positions-Regeldifferenz der 3 Motoren vor. Dafür wird zunächst feste Zielpositionen für alle 3 Motoren festgelegt, die zu Endeffektorposition von (0|0|300) führen.<br> 
Eine Änderung des Motor-P-Gains ist gleichbedeutend mit der wahrnehmbaren Steifigkeit des Motors, wenn dieser manuell aus seiner Zielposition ausgelenkt wird. Durch das adaptive Ändern des P-Gains lässt sich so das Moment des Motors in Abhängigkeit dem radialen Weg indirekt beeinflussen. <br>
Für diese Abhängigkeit wurden verschiedene Formen getestet, wie in der Abbildung unten zu sehen ist. Die aktuell im Code verwendte Abhänigikeit ist die der grünen Geraden. Diese Abhängigkeit ist entpricht auch der Intuition, da das oben beschriebne Verhalten sehr an eine Federkennlinie erinnern lässt. Als Optimierung der Geraden ist die orangene Kurve aufzufassen. Diese besteht aus der Überlagerung einer Geraden und einer e-Funktion. Die Idee hinter dieser Kombination ist, das P-Gain an den Grenzen des zulässigen Arbeitsbereichs so hoch zu setzen, dass jenes den Arbeitsbreich indirekt und v.a. weicher als eine reine if-Bedingung begrenzt. Die Parameter dieser Funktion benötigen allerdings noch Anpassung. 

[![adapKoeff](/images/adaptiveGain.png)](/images/adaptiveGain.pn)

**Disclaimer:** <br> 
Aktuell ist die adaptive Regleranpassung im Code auskommentiert. Die Funktionalität des Codes ist zwar gegeben, allerdings ist aktuell die Zuverlässigkeit im langfristigen Betrieb noch nicht gesichert. Details hierzu finden sich in den Kommentaren in der COMMOT Node (Zeilen 215-218).<br> 
Um die Zuverlässigkeit zu gewährleisten ist aktuell ein konstantes P-Gain implementiert. Dies sorgt zwar für ein generell sehr weiches Verhalten und eine gute Zentrierung in den Ursprung bzgl. der x-y-Ebene, jedoch gleichzeitig auch für eine große Ungenauigkeit bzgl. der Z-Koordinate. Diese Abweichung existiert auch mit adaptiver Regleranpassung aber in geminderter Form.

**ROS2 Struktur des Compliant-Modus:**<br>
In der Abbildung unterhalb des Absatzes ist der Datenfluss für den Compliant-Mode dargestellt. Wie bereits beim Punktnavigationsmodus erwähnt, wird auch hier der Start des Modus über das Publizieren einer *OpMod*-Nachricht im Terminal (T) in­i­ti­ie­rt, wobei mit dem Zusatz -r eine Publikationsfrequenz eingestellt werden muss. Mit dieser Frequenz wird einerseits die aktuelle Position des Endeffektors kontinuierlich berechnet und andererseits die aktuellen P-Gains für den Fall der adaptiven Regelparameter-Veränderung.<br> 
Die COMMOT Node verändert nach Empfang der Nachricht die P- und D-Gains der Motoren (entweder zu anderen Konstantwerten oder adaptiv). Die OPMOD Node stellt parallel einen service request an die COMMOT Node, um die aktuelle Position der Motoren zu erhalten. Wurden die Positionen empfangen, wird die direkte Kinematik berechnet und das Ergbnis über eine TimerCallback als Twist-Nachricht publiziert.<br>
Das Publizieren in einer TimerCallback ist notwendig, da die dem Projekt zur Verfügung stehende mobile Plattform eine gewisse Frequenz an Twist-Nachrichten braucht, um ein flüssiges Drehen der Räder zu gewährleisten. Diese Frequenz ist allerdings so hoch, dass es bei der Verwendung dieser bei der Effektorpositionsbestimmung zu Kommunikationsfehlern während des Present Position services kommt. Daher wird das Publizieren der Twist Nachricht vom Rest des Compliant-Modes mit Hilfe der TimerCallback zeitlich entkoppelt.

[![comp](/images/compliant.png)](/images/compliant.png)

Für die Aktivierung des Compliant - Modus sind folgende Schritte erforderlich
- in neuem Terminal:
```
source install/setup.bash
```
```
ros2 topic pub -r 2 OpMod delta_robot_interfaces/OpMod "{mode: 1}"
```
**Hinweis:** <br>
Die Regelparameter lassen sich in [commot_node.cpp](/src/delta_robot/src
/commot_node.cpp) in Zeile 216 (P-Gain) und 225 (D-Gain) anpassen.<br>
Die Frequenz von 2 Hz dient, wie bereits erwähnt für die kontinuierliche Berechnung der Effektorposition und nicht zum Publizieren der Twist Nachrichten. Um die Twist-Frequenz zu verstellen, muss die Periodendauer des Timers in der OPMOD Node (Zeile 50) angepasst werden.

## 5. Simulator für Delta-Roboter

Außerdem wurde während der Projektarbeit ein Simulator für den Delta-Roboter aufgebaut. Dafür wurde zu Beginn Gazebo genutzt. Da dies nun nichtmehr unterstützt wird wurde auf die neue Umgebung Iginition gewechselt. ALle notwendigen Datein sind im "delta_robot_ros2" Package abgelegt. Außerdem müssen die Schritte aus Abschnitt 4.1 durchgeführt werden, um die simulation zu starten.

Für das Starten der Iginition Umgebung ist folgender Befehl auszuführen
```
ros2 launch delta_robot_gazebo delta_robot_bringup_gazebo.launch.py
```

Für das Starten der Gazebo Umgebung ist folgender Befehl auszuführen
```
ros2 launch delta_robot_gazebo delta_robot_bringup_classic_gazebo.launch.py
```

Anmerkung: Der hier aufgebaute Simulator stellt nur ein Grundgerüst dar. Insbesondere hinsichtlich der closed-loop-Kinematik muss der Simulator noch ausgearbeitet werden. Zum Zeitpunkt der Projektarbeit wurde hierfür noch keine passende Lösung gefunden. Außerdem ist auch noch ein Skript zu schreiben, mit welchem die Arme bewegt werden können. Derzeit können die Arme des Delta Roboters nur direkt innerhalb der Simulationsumgebung über den "Joint position Controller" gesteuert werden.
