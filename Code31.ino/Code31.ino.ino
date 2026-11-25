#include <Wire.h>                         // Inclut la bibliothèque Wire pour la communication I2C
#include <LiquidCrystal_PCF8574.h>        // Pilote l'écran LCD I2C de manière stable
#include <Servo.h>                        // Inclut la bibliothèque Servo pour contrôler les moteurs
#include <SoftwareSerial.h>               // Pour la communication série
#include <DFRobotDFPlayerMini.h>          // Pour contrôler le DFPlayer Mini

LiquidCrystal_PCF8574 lcd(0x27);          // Initialise l'écran LCD à l'adresse I2C 0x27

Servo servoCouvercle;                     // Servo 1 (Couvercle / Trappe)
Servo servoTri;                           // Servo 2 (Aiguillage interne)

// --- Configuration des Broches ---
const int pinTrigCouvercle = 2;           // Ultrason 1 : Détection présence
const int pinEchoCouvercle = A0;          

const int pinInductif = 4;                // Capteur inductif de détection de métal physique

const int pinTrigBacMetal = 5;            // Trigger du bac Métal
const int pinEchoBacMetal = 6;            // Echo du bac Métal

const int pinTrigBacPlastique = 7;        // Trigger du bac Plastique
const int pinEchoBacPlastique = 8;        // Echo du bac Plastique

const int pinTrigBacAutres = A1;          // Trigger du bac Autres (sur A1)
const int pinEchoBacAutres = A2;          // Echo du bac Autres (sur A2)

// --- CONFIGURATION DES BROCHES 0 ET 1 (LIBÉRÉES DU SERIAL) ---
const int pinRelaisLampe = 0;             // Broche 0 (RX) : Relais de l'éclairage crépusculaire
const int pinLedBacPlein = 1;             // Broche 1 (TX) : LED indiquant que le bac est plein

// --- CONFIGURATION MODULE CRÉPUSCULAIRE ---
const int pinCapteurLDR = A3;             // Entrée analogique du capteur LDR (A3)

// --- SEUILS DE LUMINOSITÉ CRÉPUSCULAIRE ---
const int SEUIL_NUIT = 300;               // En dessous de cette valeur -> Nuit (Allume la lampe)
const int SEUIL_JOUR = 450;               // Au-dessus de cette valeur -> Jour (Éteint la lampe)

// Passe à true entre l'envoi de CAPTURE et la réponse de l'ESP32 :
// le crépusculaire considère alors qu'il fait nuit (la lampe reste allumée)
bool forcerNuit = false;

const int pinServoCouvercle = 9;          // Servo 1 (Alimentation externe 5V)
const int pinServoTri = 3;                // Servo 2 (Alimentation externe 5V)

const int pinRX_DFP = 10;                 // RX de l'Arduino (à relier au TX du DFPlayer)
const int pinTX_DFP = 11;                 // TX de l'Arduino (à relier au RX du DFPlayer via 1k)

// --- COMMUNICATION SÉRIE AVEC L'ESP32-CAM ---
SoftwareSerial espSerial(12, 13);         

// --- Hauteur interne de chaque bac (en cm) ---
const float hauteurMaxBacPlastique = 90.0;   // Bac plastique
const float hauteurMaxBacMetal     = 65.0;   // Bac métal
const float hauteurMaxBacAutres    = 65.0;   // Bac autres

// --- Son "rien n'a été détecté" dans le DFPlayer ---
// Le fichier doit s'appeler 0012.mp3 dans le dossier MP3 de la carte SD (change le numéro si besoin)
const int SON_RIEN_DETECTE = 12;

// --- Initialisation des objets Audio ---
SoftwareSerial monSerialAudio(pinRX_DFP, pinTX_DFP); 
DFRobotDFPlayerMini monDFPlayer;                        

// Variables globales pour le niveau des 3 bacs
int remplissageMetal = 0;
int remplissagePlastique = 0;
int remplissageAutres = 0;

// Détection du vidage d'un bac (même s'il n'a pas atteint 80%) :
// re-mesure périodique, mais la valeur affichée ne change que si le niveau BAISSE nettement
const unsigned long intervalleControleVidage = 5000;   // ms entre deux contrôles
const int SEUIL_VIDAGE = 10;                           // baisse minimale (en points de %) pour valider un vidage
unsigned long dernierControleVidage = 0;

// Prototypage des fonctions
void gererEclairageCrepusculaire();
void attenteAvecCrepusculaire(unsigned long millisecondes);
void bougerServoDoucement(Servo &monServo, int angleCible);
void calculerNiveauxBacs();
void afficherNiveaux();
bool unBacEstPlein();
void verifierVidageBacs();
void viderBufferEsp();
bool attendreReponseEsp(String &reponseSortie, unsigned long timeoutMs);

void setup() {
  Wire.begin();
  Wire.setClock(100000);                    

  lcd.begin(16, 2);                          
  lcd.setBacklight(255);                     
  
  lcd.setCursor(0, 0);
  lcd.print("   BIENVENUE A");
  lcd.setCursor(0, 1);
  lcd.print("ERIC'S SMARTBIN");
  
  // --- Configuration des broches 0 et 1 ---
  pinMode(pinRelaisLampe, OUTPUT);
  pinMode(pinLedBacPlein, OUTPUT);
  pinMode(pinCapteurLDR, INPUT);
  
  digitalWrite(pinRelaisLampe, LOW);      // Relais éteint au démarrage
  digitalWrite(pinLedBacPlein, LOW);      // LED bac plein éteinte au démarrage

  // --- Initialisation des ports séries ---
  espSerial.begin(9600);                  // Pour communiquer avec l'ESP32-CAM

  // --- Initialisation Audio ---
  monSerialAudio.begin(9600);                 
  if (monDFPlayer.begin(monSerialAudio, false, true)) {
    monDFPlayer.volume(30);                     
    monDFPlayer.playMp3Folder(1);         // Son d'accueil au démarrage (son 1)
  }
  
  attenteAvecCrepusculaire(3000);                            
  lcd.clear();
        
  pinMode(pinTrigCouvercle, OUTPUT);         
  pinMode(pinEchoCouvercle, INPUT);  
          
  pinMode(pinInductif, INPUT_PULLUP);        

  // Configuration des 3 capteurs de niveau de bacs
  pinMode(pinTrigBacMetal, OUTPUT);          
  pinMode(pinEchoBacMetal, INPUT);           
  pinMode(pinTrigBacPlastique, OUTPUT);      
  pinMode(pinEchoBacPlastique, INPUT);       
  pinMode(pinTrigBacAutres, OUTPUT);         
  pinMode(pinEchoBacAutres, INPUT);          

  servoCouvercle.attach(pinServoCouvercle);  
  servoTri.attach(pinServoTri);              

  // --- ÉTATS INITIAUX : Les deux servomoteurs sont positionnés à 90° ---
  servoCouvercle.write(90);                  
  attenteAvecCrepusculaire(500);                                
  servoTri.write(84);                        
  attenteAvecCrepusculaire(100);

  // CORRECTION OSCILLATION : Libération des servos au repos
  servoCouvercle.detach();
  servoTri.detach();

  // --- CALCUL INITIAL DES NIVEAUX DE REMPLISSAGE (une seule fois au démarrage) ---
  calculerNiveauxBacs();
}

void loop() {
  // --- 0. GESTION CONTINUE DE L'ÉCLAIRAGE CRÉPUSCULAIRE ---
  gererEclairageCrepusculaire();

  // --- 1. Mesure de la présence utilisateur (Ultrason 1) ---
  long dureePres, distancePres;
  digitalWrite(pinTrigCouvercle, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrigCouvercle, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrigCouvercle, LOW);
  dureePres = pulseIn(pinEchoCouvercle, HIGH, 30000); 
  distancePres = (dureePres / 2) / 29.1;

  // Détection fixée à 24 cm
  if (distancePres <= 24 && distancePres > 0) { 
    
    // Si un bac est marqué plein, on re-mesure d'abord : il a peut-être été vidé depuis
    if (unBacEstPlein()) {
      calculerNiveauxBacs();
      dernierControleVidage = millis();
    }

    // Sécurité : Si l'un des 3 bacs atteint ou dépasse 80%
    if (unBacEstPlein()) { 
      lcd.setCursor(0, 0);
      lcd.print("BAC PLEIN !     ");
      lcd.setCursor(0, 1);
      lcd.print("DEPOT IMPOSSIBLE");
      
      if (remplissagePlastique >= 80 && remplissageMetal >= 80 && remplissageAutres >= 80) {
        monDFPlayer.playMp3Folder(11);       // Tous les bacs sont pleins
      } else if (remplissagePlastique >= 80) {
        monDFPlayer.playMp3Folder(8);        // Plastique plein
      } else if (remplissageMetal >= 80) {
        monDFPlayer.playMp3Folder(9);        // Métal plein
      } else if (remplissageAutres >= 80) {
        monDFPlayer.playMp3Folder(10);       // Autres plein
      }
      
      // CLIGNOTEMENT DE LA LED BAC PLEIN SUR LA BROCHE 1
      for (int i = 0; i < 8; i++) {
        digitalWrite(pinLedBacPlein, HIGH);
        attenteAvecCrepusculaire(250);
        digitalWrite(pinLedBacPlein, LOW);
        attenteAvecCrepusculaire(250);
      }
      
      lcd.clear();
    } 
    else {
      // --- Traitement du dépôt ---
      lcd.setCursor(0, 0);
      lcd.print("ANALYSE MATIERE ");
      
      monDFPlayer.playMp3Folder(2);          
      
      String resultatEsp = "";
      bool cycleAnnule = false;   // Passe à true si rien n'a été détecté

      // Fenêtre d'analyse de 4 secondes : on surveille le capteur inductif PENDANT toute la durée
      bool metalDetecte = false;
      unsigned long debutAnalyse = millis();
      while (millis() - debutAnalyse < 4000) {
        gererEclairageCrepusculaire();
        if (digitalRead(pinInductif) == HIGH) {
          metalDetecte = true;
        }
      }

      // VÉRIFICATION DU CAPTEUR INDUCTIF
      if (metalDetecte) {
        // --- CAS 1 : DÉCHET MÉTALLIQUE ---
        lcd.setCursor(0, 0);
        lcd.print("DECHET: METAL   ");
        monDFPlayer.playMp3Folder(3);        // Son 3 : Métal détecté
        
        bougerServoDoucement(servoTri, 160);         
        attenteAvecCrepusculaire(2000);                                
        bougerServoDoucement(servoCouvercle, 40);   
        attenteAvecCrepusculaire(3000);                                

        // Repli
        bougerServoDoucement(servoCouvercle, 90);
        attenteAvecCrepusculaire(1000);                                    
        bougerServoDoucement(servoTri, 84);
      } 
      else {
        // --- CAS 2 : PAS DE MÉTAL -> DOUBLE ENVOI CAPTURE À L'ESP32-CAM ---
        // On envoie CAPTURE une 1re fois (réponse ignorée pour purger l'ancien état),
        // puis une 2e fois et on ne conserve que la 2e réponse.
        espSerial.listen();   // On écoute l'ESP32 (et plus le DFPlayer)
        viderBufferEsp();

        forcerNuit = true;                // Considère qu'il fait nuit pendant la capture/analyse
        gererEclairageCrepusculaire();

        // 1er envoi CAPTURE (réponse ignorée)
        String premiereReponse = "";
        espSerial.println("CAPTURE");
        attendreReponseEsp(premiereReponse, 10000);
        viderBufferEsp();

        // 2e envoi CAPTURE (seule réponse retenue)
        String deuxiemeReponse = "";
        espSerial.println("CAPTURE");
        bool reponseRecue = attendreReponseEsp(deuxiemeReponse, 10000);
        viderBufferEsp();

        forcerNuit = false;               // Fin de la phase d'analyse : retour à la LDR réelle
        gererEclairageCrepusculaire();

        monSerialAudio.listen();   // On redonne l'écoute au DFPlayer

        resultatEsp = reponseRecue ? deuxiemeReponse : "";

        if (reponseRecue && resultatEsp == "plastique") {
          // --- SOUS-CAS : PLASTIQUE ---
          lcd.setCursor(0, 0);
          lcd.print("DECHET:PLASTIQUE");
          monDFPlayer.playMp3Folder(6);        // Son 6 : Plastique détecté
          
          bougerServoDoucement(servoCouvercle, 170);  
          attenteAvecCrepusculaire(4000);                                

          // Repli
          bougerServoDoucement(servoCouvercle, 90);
          attenteAvecCrepusculaire(1000);                                    
          bougerServoDoucement(servoTri, 84);
        } 
        else if (reponseRecue && resultatEsp == "autres") {
          // --- SOUS-CAS : AUTRES DÉCHETS (Papier / Carton) ---
          lcd.setCursor(0, 0);
          lcd.print("DECHET: AUTRES  ");
          monDFPlayer.playMp3Folder(7);        // Son 7 : Autres déchets détectés
          
          bougerServoDoucement(servoTri, 15);          
          attenteAvecCrepusculaire(2000);                                
          bougerServoDoucement(servoCouvercle, 40);   
          attenteAvecCrepusculaire(3000);                                

          // Repli
          bougerServoDoucement(servoCouvercle, 90);
          attenteAvecCrepusculaire(1000);                                    
          bougerServoDoucement(servoTri, 84);
        } 
        else {
          // --- SOUS-CAS : RIEN / BACKGROUND OU PAS DE RÉPONSE ---
          cycleAnnule = true;
          lcd.setCursor(0, 0);
          lcd.print("RIEN DETECTE    ");
          lcd.setCursor(0, 1);
          lcd.print("CYCLE ANNULE    ");

          if (reponseRecue && resultatEsp == "rien") {
            monDFPlayer.playMp3Folder(SON_RIEN_DETECTE);
          }
          attenteAvecCrepusculaire(3000);
        }
      }

      lcd.clear();

      // Mise à jour du niveau + sons de niveau seulement si un déchet a bien été trié
      if (!cycleAnnule) {
        monDFPlayer.playMp3Folder(4);          // Son 4 : Mise à jour niveau
        calculerNiveauxBacs();                 
        attenteAvecCrepusculaire(2500);                            
        
        monDFPlayer.playMp3Folder(5);          // Son 5 : Merci pour le tri
      }
    }
  }

  // --- Mode Veille ---
  if (millis() - dernierControleVidage > intervalleControleVidage) {
    verifierVidageBacs();
    dernierControleVidage = millis();
  }

  afficherNiveaux();

  attenteAvecCrepusculaire(500);                                         
}

// =======================================================================
// FONCTIONS UTILITAIRES
// =======================================================================

bool attendreReponseEsp(String &reponseSortie, unsigned long timeoutMs) {
  unsigned long debut = millis();
  reponseSortie = "";

  while (millis() - debut < timeoutMs) {
    gererEclairageCrepusculaire();
    if (espSerial.available() > 0) {
      String ligne = espSerial.readStringUntil('\n');
      ligne.trim();
      if (ligne == "plastique" || ligne == "autres" || ligne == "rien") {
        reponseSortie = ligne;
        return true;
      }
    }
  }
  return false;
}

void viderBufferEsp() {
  unsigned long t0 = millis();
  while (millis() - t0 < 30) {
    while (espSerial.available() > 0) {
      espSerial.read();
      t0 = millis();
    }
  }
}

/**
 * Fonction : gererEclairageCrepusculaire
 * Mesure la luminosité sur A3 et contrôle le relais de la lampe sur la broche 0
 */
void gererEclairageCrepusculaire() {
  int luminosite = analogRead(pinCapteurLDR);

  if (forcerNuit) {
    luminosite = 1023;
  }

  if (luminosite < SEUIL_NUIT) {
    digitalWrite(pinRelaisLampe, HIGH); // Mettre LOW si le relais est Active LOW
  } 
  else if (luminosite > SEUIL_JOUR) {
    digitalWrite(pinRelaisLampe, LOW);  // Mettre HIGH si le relais est Active LOW
  }
}

/**
 * Fonction : attenteAvecCrepusculaire
 * Remplace delay() pour garder une vérification de l'éclairage active pendant les pauses
 */
void attenteAvecCrepusculaire(unsigned long millisecondes) {
  unsigned long debut = millis();
  while (millis() - debut < millisecondes) {
    gererEclairageCrepusculaire();
  }
}

void bougerServoDoucement(Servo &monServo, int angleCible) {
  if (&monServo == &servoCouvercle) {
    monServo.attach(pinServoCouvercle);
  } else if (&monServo == &servoTri) {
    monServo.attach(pinServoTri);
  }
  
  int angleActuel = monServo.read(); 
  
  if (angleActuel < angleCible) {
    for (int pos = angleActuel; pos <= angleCible; pos++) {
      monServo.write(pos);
      attenteAvecCrepusculaire(5);
    }
  } else {
    for (int pos = angleActuel; pos >= angleCible; pos--) {
      monServo.write(pos);
      attenteAvecCrepusculaire(5);
    }
  }

  attenteAvecCrepusculaire(100);
  monServo.detach(); 
}

bool unBacEstPlein() {
  return (remplissagePlastique >= 80 || remplissageMetal >= 80 || remplissageAutres >= 80);
}

void verifierVidageBacs() {
  int ancienMetal = remplissageMetal;
  int ancienPlastique = remplissagePlastique;
  int ancienAutres = remplissageAutres;

  calculerNiveauxBacs();

  if (remplissageMetal > ancienMetal - SEUIL_VIDAGE)         remplissageMetal = ancienMetal;
  if (remplissagePlastique > ancienPlastique - SEUIL_VIDAGE) remplissagePlastique = ancienPlastique;
  if (remplissageAutres > ancienAutres - SEUIL_VIDAGE)       remplissageAutres = ancienAutres;
}

void afficherNiveaux() {
  lcd.setCursor(0, 0);
  lcd.print("MET:");
  lcd.print(remplissageMetal);
  lcd.print("% PLAST:");
  lcd.print(remplissagePlastique);
  lcd.print("%   ");
    
  lcd.setCursor(0, 1);
  lcd.print("AUTRES   : ");
  lcd.print(remplissageAutres);
  lcd.print("%   ");
}

void calculerNiveauxBacs() {
  long durMet, distMet;
  digitalWrite(pinTrigBacMetal, LOW);  delayMicroseconds(2);
  digitalWrite(pinTrigBacMetal, HIGH); delayMicroseconds(10);
  digitalWrite(pinTrigBacMetal, LOW);
  durMet = pulseIn(pinEchoBacMetal, HIGH, 30000);
  distMet = (durMet / 2) / 29.1;

  long durPlas, distPlas;
  digitalWrite(pinTrigBacPlastique, LOW);  delayMicroseconds(2);
  digitalWrite(pinTrigBacPlastique, HIGH); delayMicroseconds(10);
  digitalWrite(pinTrigBacPlastique, LOW);
  durPlas = pulseIn(pinEchoBacPlastique, HIGH, 30000);
  distPlas = (durPlas / 2) / 29.1;

  long durAut, distAut;
  digitalWrite(pinTrigBacAutres, LOW);  delayMicroseconds(2);
  digitalWrite(pinTrigBacAutres, HIGH); delayMicroseconds(10);
  digitalWrite(pinTrigBacAutres, LOW);
  durAut = pulseIn(pinEchoBacAutres, HIGH, 30000);
  distAut = (durAut / 2) / 29.1;

  if (distMet > hauteurMaxBacMetal) distMet = hauteurMaxBacMetal;
  if (distPlas > hauteurMaxBacPlastique) distPlas = hauteurMaxBacPlastique;
  if (distAut > hauteurMaxBacAutres) distAut = hauteurMaxBacAutres;

  remplissageMetal = ((hauteurMaxBacMetal - distMet) / hauteurMaxBacMetal) * 100;
  remplissagePlastique = ((hauteurMaxBacPlastique - distPlas) / hauteurMaxBacPlastique) * 100;
  remplissageAutres = ((hauteurMaxBacAutres - distAut) / hauteurMaxBacAutres) * 100;

  if (remplissageMetal < 0) remplissageMetal = 0;
  if (remplissagePlastique < 0) remplissagePlastique = 0;
  if (remplissageAutres < 0) remplissageAutres = 0;
}
