# OBDCanLogger

Bibliotheque Arduino pour ESP32-C3 + MCP2515 qui reproduit le principe du logger `GKFlasher -l --logger`:

- ouverture d'une session KWP2000 sur CAN (`0x7E0` -> `0x7E8`);
- transport ISO-TP minimal pour requetes courtes et reponses multi-trames;
- keepalive `TesterPresent`;
- polling `ReadDataByLocalIdentifier(0x21)` sur le bloc `0x01`;
- sortie CSV sur le port serie.

## Dependances

Installe la librairie Arduino `mcp2515` compatible avec `struct can_frame` et `MCP2515`.

## Cablage type ESP32-C3 SuperMini -> MCP2515

- `3V3` -> `VCC` si ton module MCP2515 accepte 3.3 V logique
- `GND` -> `GND`
- `GPIO4` -> `SCK`
- `GPIO6` -> `MISO`
- `GPIO7` -> `MOSI`
- `GPIO10` -> `CS`
- `GPIO5` -> `INT` optionnel

Adapte les pins SPI/CS a ton montage reel.

## Utilisation

Le sketch principal [`OBDBoard.ino`](./OBDBoard.ino) montre l'usage:

1. configure CAN, IDs et pin `CS`;
2. appelle `logger.begin()`;
3. dans `loop()`, appelle `kwp.loop()` puis `logger.poll(Serial)`.

## Test materiel

Un sketch de validation rapide est disponible dans [`examples/Mcp2515HardwareTest/Mcp2515HardwareTest.ino`](./examples/Mcp2515HardwareTest/Mcp2515HardwareTest.ino).

Il fait deux choses:

1. test SPI + MCP2515 en mode loopback interne, sans dependre du bus CAN du vehicule;
2. passe en mode normal et ecoute le bus 10 secondes pour verifier que le TJA1050 et le cablage CAN voient du trafic.

Si le loopback passe mais qu'aucune trame n'apparait en mode normal, le probleme est cote bus/transceiver/vitesse CAN plutot que cote SPI.

Un second sketch de sonde diagnostic est disponible dans [`examples/KwpDiagProbe/KwpDiagProbe.ino`](./examples/KwpDiagProbe/KwpDiagProbe.ino).

Il envoie des requetes simples vers `0x7E0` et attend une reponse sur `0x7E8`:

- `0x81` StartCommunication
- `0x3E 0x01` TesterPresent
- `0x01 0x00` requete OBD-II standard

Ce sketch est configure avec les pins:

- `SCK=GPIO4`
- `MISO=GPIO5`
- `MOSI=GPIO6`
- `CS=GPIO7`

## Stream CAN moteur

Un sketch de streaming temps reel est disponible dans [`examples/CanEngineStream/CanEngineStream.ino`](./examples/CanEngineStream/CanEngineStream.ino).

Il decode les trames `0x316` et `0x329` et sort sur le port serie des lignes du type:

```text
"rpm"=2456;"coolant_temp"=45.0;"vehicle_speed"=0;
```

Champs inclus:

- `key_on`
- `rpm`
- `vehicle_speed`
- `coolant_temp`
- `pedal_position`
- `atmospheric_pressure`
- `indexed_engine_torque`
- `indicated_engine_torque`
- `theoretical_engine_torque`
- `torque_losses`

Le composant reutilisable [`src/EngineCanStream.h`](./src/EngineCanStream.h) reprend la meme logique sous forme de classe et ajoute une partie des trames du `DBC` Tiburon SIMK4x:

- `0x316` (`DME1`)
- `0x329` (`DME2`)
- `0x545` (`DME4`)
- `0x2A0` (`DME5`)

Le sketch [`examples/CanEngineStreamFull/CanEngineStreamFull.ino`](./examples/CanEngineStreamFull/CanEngineStreamFull.ino) publie notamment:

- `rpm`
- `vehicle_speed`
- `coolant_temp`
- `pedal_position`
- `battery_voltage`
- `fuel_consumption`
- `intake_air_temp`
- `immobilizer_authenticated`
- `mil_active`
- `immobilizer_enabled`

## Limites actuelles

- les requetes ISO-TP emises sont limitees a 7 octets, ce qui couvre les commandes logger KWP utilisees ici;
- les decodeurs inclus reproduisent la table `flasher/logging.py` pour `ReadDataByLocalIdentifier(0x01)`;
- le `DBC` n'est pas encore importe automatiquement: les champs utiles sont codes en dur dans `EngineCanStream`;
- si ton ECU utilise d'autres IDs CAN, un autre debit CAN, ou d'autres LIDs, il faut adapter la configuration ou la table de definitions.
