# ERICS_SMARTBIN
# ERICS_SMARTBIN

Système de poubelle intelligente (plastique, métal, papier et autres), développé dans le cadre dU projet de fin d'études en génie électrique à l'Université Libre des Pays des Grands Lacs (ULPGL) — Faculté des Sciences et Technologies Appliquées (FSTA), Goma.

## Architecture du dépôt

* **`Code31.ino/`** : Programme principal de commande sur Arduino Uno (gestion du tri, du capteur inductif, des servomoteurs, de l'affichage, du retour audio DFPlayer Mini et de l'éclairage crépusculaire).
* **`smartbin_esp32cam/`** : Programme embarqué sur ESP32-CAM pour la capture d'image, l'éclairage d'appoint et l'inférence de classification visuelle (Edge Impulse FOMO) avec communication série vers l'Arduino.
* **`Collect_Images_for_EdgeImpulse/`** : Scripts ou fichiers dédiés à la collecte et à la préparation du jeu d'images pour l'entraînement du modèle.

## Principe de fonctionnement

1. Détection de la présence d'un déchet et lancement de la séquence d'analyse.
2. Vérification de la nature métallique via le capteur inductif (avec filtrage logiciel contre les parasites et la mobilité de l'objet).
3. En cas d'objet non métallique, déclenchement d'une double capture sur l'ESP32-CAM afin de purger l'état précédent et obtenir une classification fraîche.
4. Orientation mécanique vers le compartiment correspondant et signalisation associée.

## Auteur

**Eric Mushagalusa Mapendano**  
Faculté des Sciences et Technologies Appliquées (FSTA) — ULPGL, Goma (RDC)

