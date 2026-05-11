# README.md — Slave LoRa arrosage avec OTA BLE

## But

Ce projet est le firmware **slave** d’un node LoRa d’arrosage.

Le node :
- dialogue avec un **master LoRa**
- exécute des commandes de supervision et d’arrosage
- mesure le débit via compteur d’impulsions
- supporte une mise à jour **OTA BLE DFU** à la demande

Le protocole embarqué ne dépend **pas d’une RTC** ni d’un epoch absolu.
Les horodatages doivent être gérés côté backend ou superviseur.

---

## Fonctionnement général

En mode normal, le slave :

1. démarre la radio LoRa
2. envoie périodiquement un `HELLO`
3. attend un `ACK_HELLO` du master
4. ouvre une courte fenêtre pour recevoir une commande optionnelle
5. exécute la commande si présente
6. retourne ensuite en attente / cycle normal

Le node peut aussi basculer en **mode OTA BLE** :
- à la demande du master
- après reboot spécial
- pour permettre une mise à jour firmware avec **nRF DFU**

---

## Identité du node

L’identifiant du node est défini dans :

```cpp
static constexpr uint8_t NODE_ID = X;
```

Cet identifiant est utilisé :
- dans le protocole LoRa
- dans les retours MQTT du master
- dans le nom BLE OTA

Nom BLE OTA du node :

```text
NODE_<id>_OTA
```

Exemple :

```text
NODE_3_OTA
```

---

## Commandes supportées

Le slave reçoit les commandes du master via LoRa.

### Commandes de base
- `ping`
- `bat`
- `status`
- `text`
- `reboot`
- `ota`

### Commandes LED
- `led on`
- `led off`

### Commandes période
- `period <seconds>`

### Commandes vanne
- `valve open`
- `valve close`

### Commandes arrosage
- `water <seconds>`
- `abort`
- `wstatus`

---

## Description fonctionnelle des commandes

### `ping`
Vérifie que le node répond.

Retour attendu :
- ACK de succès ou échec

### `bat`
Retourne la batterie en millivolts.

### `status`
Retourne l’état général du node :
- batterie
- flags d’état
- uptime

### `text`
Retourne ou traite un petit message texte de test/debug.

### `reboot`
Redémarre le node.

### `ota`
Fait redémarrer le node en mode **BLE OTA DFU**.

### `led on` / `led off`
Commande la LED locale du node.

### `period <seconds>`
Modifie la période de cycle / sommeil du node selon votre logique applicative.

### `valve open`
Ouvre la vanne.

### `valve close`
Ferme la vanne.

### `water <seconds>`
Démarre un arrosage temporisé.

Effets :
- remise à zéro du compteur flow
- ouverture vanne
- comptage des pulses pendant l’arrosage
- fermeture automatique à la fin

### `abort`
Arrête immédiatement l’arrosage en cours.

### `wstatus`
Retourne l’état détaillé de l’arrosage :
- vanne ouverte ou non
- arrosage actif ou non
- durée programmée
- temps restant
- `flowPulses`
- `litres`

---

## Watering / débitmètre

Le module watering fonctionne à partir :
- de l’état de vanne
- d’une durée d’arrosage
- d’un compteur d’impulsions sur entrée flow

### Comptage des pulses
Le compteur `flowPulses` est en :

```cpp
uint32_t
```

Donc il n’est pas limité à `65535`.

### Litres
La conversion en litres reste en :

```cpp
uint16_t
```

### Politique de comptage
Le comptage flow est actif uniquement lorsque la vanne est ouverte :
- interruption attachée dans `openValve()`
- interruption détachée dans `closeValve()`

Cela évite de compter des pulses hors période utile.

---

## Structure des données watering

Les retours watering utilisent le format logique suivant :

- `valveOpen` : 1 octet
- `wateringActive` : 1 octet
- `durationSec` : 2 octets
- `remainingSec` : 2 octets
- `flowPulses` : 4 octets
- `litres` : 2 octets

Donc :
- `flowPulses` = **uint32_t**
- `litres` = **uint16_t**

---

## OTA BLE DFU

## Principe
Le node ne garde **pas le BLE actif en permanence**.

En fonctionnement normal :
- pas de BLE OTA
- consommation plus faible
- fonctionnement LoRa normal

Le BLE OTA n’est activé que :
- après une commande OTA
- puis un reboot spécial

## Déclenchement OTA
Le master envoie une commande LoRa OTA au node.

Le node :
1. répond avec un ACK
2. pose un flag de boot OTA
3. redémarre
4. entre en mode BLE OTA

## Au reboot OTA
Au démarrage, `main.cpp` teste le flag OTA :

- si flag absent → mode normal LoRa
- si flag présent → mode OTA BLE

Dans ce cas :
- la radio LoRa n’est pas lancée
- la logique watering normale n’est pas lancée
- seul le mode BLE OTA est actif

## Nom BLE OTA
Le périphérique BLE annoncé par le node est :

```text
NODE_<id>_OTA
```

Exemple :

```text
NODE_3_OTA
```

## Mise à jour
La mise à jour se fait avec :
- l’application **nRF DFU**
- un fichier `firmware.zip` compatible bootloader

## Fin de mode OTA
Si aucune mise à jour n’est effectuée pendant la fenêtre OTA :
- le timeout OTA expire
- le node redémarre
- il revient en mode normal

---

## Absence volontaire de RTC / epoch embarqué

Le slave ne dépend pas d’une heure absolue.

### Choix assumé
Le projet ne transporte pas :
- d’epoch RTC dans le protocole
- de synchronisation horaire embarquée
- de timestamp absolu fourni par le master

### Pourquoi
Ce choix rend le système :
- plus simple
- plus robuste
- moins dépendant du LTE / RTC / parsing date
- plus facile à maintenir

### Conséquence
Les messages doivent être horodatés côté :
- broker MQTT
- backend
- superviseur
- Jeedom / Home Assistant / base de données

---

## Flux normal du protocole LoRa

Cycle standard :

1. le slave envoie `HELLO`
2. le master répond `ACK_HELLO`
3. le slave ouvre une fenêtre de réception commande
4. si une commande est reçue :
   - elle est validée
   - elle est exécutée
   - un ACK de commande est retourné
5. le cycle reprend

---

## Conseils d’exploitation

### Pour l’OTA
- vérifier que le bon bootloader OTAFIX est installé sur la bonne carte
- utiliser nRF DFU avec le `firmware.zip`
- vérifier le nom BLE `NODE_<id>_OTA`

### Pour le débitmètre
- vérifier la bonne constante `FLOW_PULSES_PER_LITRE`
- documenter clairement la valeur utilisée selon le capteur réel
- distinguer mode test PWM et capteur réel si nécessaire

### Pour le dépannage
En cas de problème, vérifier d’abord :
- le boot normal LoRa
- la réception de `HELLO`
- les ACK de commandes
- l’entrée en mode OTA
- la visibilité BLE du node

---

## Documentation Doxygen

Le projet contient des commentaires Doxygen dans les fichiers applicatifs :

- `include/ble_ota.h`
- `include/watchdog_simple.h`
- `include/board_config.h`
- `include/watering.h`
- `include/lora_slave.h`
- `src/main.cpp`
- `src/ble_ota.cpp`
- `src/watchdog_simple.cpp`
- `src/watering.cpp`
- `src/lora_slave.cpp`

Les fichiers de support carte et exemples situés dans `RAK_PATCH_V2/` ne font pas partie de la documentation applicative.

### Installation

Sur macOS :

```bash
brew install doxygen
```

Optionnel, pour les graphes d’appel et diagrammes :

```bash
brew install graphviz
```

### Génération HTML

Depuis le dossier du projet :

```bash
cd /Users/jacques/Documents/PlatformIO/Projects/lora_p2p_master_slaves/pour_validations/unifiedboards_watering_lora_p2p_slave_ble_ota_dfu_noRTC_clean_fuites
doxygen Doxyfile
```

Puis ouvrir la documentation :

```bash
open html/index.html
```

### Options importantes du `Doxyfile`

Pour afficher les méthodes privées, variables membres, constantes, macros et fonctions statiques, le `Doxyfile` doit contenir au minimum :

```ini
INPUT                  = src include
RECURSIVE              = NO
FILE_PATTERNS          = *.cpp *.h

GENERATE_HTML          = YES
GENERATE_LATEX         = NO

EXTRACT_ALL            = YES
EXTRACT_PRIVATE        = YES
EXTRACT_STATIC         = YES
EXTRACT_LOCAL_CLASSES  = YES
EXTRACT_LOCAL_METHODS  = YES

HIDE_UNDOC_MEMBERS     = NO
HIDE_UNDOC_CLASSES     = NO

ENABLE_PREPROCESSING   = YES
MACRO_EXPANSION        = YES
EXPAND_ONLY_PREDEF     = NO

SOURCE_BROWSER         = YES
INLINE_SOURCES         = NO
REFERENCED_BY_RELATION = YES
REFERENCES_RELATION    = YES
GENERATE_TREEVIEW      = YES
```

### Où trouver les descriptions

Dans l’interface HTML Doxygen :

- les classes principales sont dans `Classes`
- les fonctions globales sont dans `Files`
- les constantes et macros sont visibles dans les pages de fichiers si `MACRO_EXPANSION` et `EXTRACT_ALL` sont actifs
- les membres privés sont visibles si `EXTRACT_PRIVATE = YES`
- les variables statiques et fonctions statiques sont visibles si `EXTRACT_STATIC = YES`

Les variables locales déclarées à l’intérieur d’une fonction ne sont pas documentées comme API par Doxygen. Pour les inspecter, utiliser l’onglet source généré par :

```ini
SOURCE_BROWSER = YES
```

---

## Résumé

Le slave fournit :
- un protocole LoRa simple et robuste
- une logique d’arrosage temporisé
- un comptage débitmètre sur `uint32_t`
- un mode OTA BLE activé à la demande
- aucune dépendance à une RTC embarquée

C’est un node volontairement simplifié pour privilégier :
- la robustesse
- la lisibilité
- la maintenance
- la faible dépendance à des services externes
