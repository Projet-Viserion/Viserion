# GeiCar Project

The GeiCar project is a project carried out by students at [INSA Toulouse](http://www.insa-toulouse.fr/fr/index.html). This project consists in developing the software of a autonomous car in order to carry out different missions. Several projects are exposed on the [official website] (https://sites.google.com/site/projetsecinsa/).

This repository is intended to provide a basis for students starting a new project on the GeiCar. The present code as well as the documentation is the result of the combination of the various projects carried out by:

* DJELASSI Amandine, ANGLERAUD Alexandre, MAYIMA Amandine, MOUSTAID Jamal, PHAN Vu Hoang, RAKOTOPARE Marcy
* GUGLIELMINOTTI-GHERMOTTI Pierre, JACQUES Lucas, GERMAIN Clement, VERTUEUX Noemie, VIGUIER Florian, ZERBIB Lea.
* LIU Zepeng, POUVREAU Thomas, GOFRE Jauri, GRANIE Guillaume, HUNEAU Louis, LOUIS Axel.
* SELLAM Nabil, ZHANG Guang Xue, BONIN Antoine, CHADUC Clement, DUCONGE Kevin, REBOUCAS MAIA  Lucas.
* BENAZECH Alexandre, BRIARD Sébastien, CASSAGNE Ludovic, DOTHEE Solène, SARTHOU Guillaume.
* COMBATTELLI Julien, EGRETEAU Corentin, DE BRITO Guillaume, DUPERON Liliane, CHATAIGNON Mickael.
* DEBAILLEUX Margaux, COMBES Jean, FARO Mathias, FAYARD Baptiste, GANDOU Thomas.
* GISSOT Lucas, LUBAT Eric, LARSSON Léo, LOUPIAS Delia, TAMIMO Volamiary.
* MEDRANO Johan, MIRAULT Clara, MONNERET Vivien, PORTES Quentin, SHIN Sohun, IGLESIS Enzo.
* ANTUNES COELHO Carolina, CHATELAIN Vincent, JOURDE Tanguy, PUECHMAILLE David, SIRGABSOU Yandika
* KIM EunHwan, STENKULA Ninni, VIGNOLLES Amaury, TAHIRI Abdelilah, OJEDA Leonardo, PONCY Jonathan
* AMARIR Azeddine, BOUHASSOUM Assya, DELGA Baptiste, LASGUIGNES Thibaud, LEMAÎTRE Aurélien, PARIS Christine
* BELLET Valentine, PUECH Camille, MUREDDU Antoine, NUSSLI Gabriel, SADIK Anass

The platform is (or was) developped and maintained by :

* DI MERCURIO Sébastien
* LOMBARD Emmanuel
* MARTIN José
* SENANEUCH Lucien


The projects are (or were) surpervised by:

* ACCO Pascal
* CAMPS Frédéric
* CHANTHERY Elodie
* DELAUTIER Sébastien
* GAUDEL Quentin
* HLADIK Pierre-Emmanuel
* LESUR Thibault
* LE BOTLAN Didier
* SUBIAS Audine

## Quick User Guide
###Turn the car on and off
* To turn on the car:
  * Toggle the red button to bring the power.
  * Press the START push button (hold it down for a short while).
  * Flip the switch to the right of the dashboard to power the Raspberry.

* To turn off the car:
	* Use the red button as a switch to turn off the power.


###Compile and load
* `git clone https://github.com/pehladik/geicar.git`
* For the embedded software on the Nucleo, use Keil to compile the project and load the code on the card. __Warning__, to load the code on the card, you must hold down the START button during the operation (see details in the documentation).
* For the embedded software on the discovery, use Attolic to compile and load the code (see more details in the documentation).
* For the embedded demo code on the Raspberry, you only need to load the file `server.py`. To do this, the easiest way is to connect in ssh to the Raspberry and copy the file there (for example with sftp or scp). Run this demo with the command `python3 server.py`. You can use a basic interface in C# to use the demo. This interface is described in the documentation.
