# Colibrino
Esse é um projeto open-source de uma versao "Faça você mesmo" do nosso mouse de cabeça o Colibri! O objetivo principal dele é promover acessibilidade para pessoas com deficiências físicas do como tetraplegia, artrogripose, amputações e parilisia cerebral.

# Materiais:
# Versao com fio
- Arduino Leonardo
- MPU6050
- Cabo com pelo menos 4 vias
- Placa padrão ou protoboard
- Led
- Resistor 1k
- Conector P2 macho
# Versao Bluetooth
(Ainda em desenvolvimento)
- ESP32
- Bateria 3.7v
- Carregador de bateria 
- Demais componente usados na versão com fio exceto o Arduino 

# Instruções de montagem:
1º Faça as conexões como a foto abaixo numa protoboard ou use uma placa padrão para soldar os componentes nela.
2º Solde os fios que vão na plaquinha do MPU6050 e conecte-os na sua placa(Arduino ou ESP32)
3º Fixe a plaquinha do MPU6050 em algum óculos
4º Grave o código na placa que estiver utilizando (Arduino->HeadMouse_ino.ino, ESP32 HeadMouse_ESP.ino) utilizando o ambiente de desenvolvimento do Arduino devidamente configurado
5º Use e aproveite ;)

# Materiais de referencia:
https://www.youtube.com/watch?v=4sIkW7wogrE
https://github.com/witnessmenow/arduino-switcheroonie/blob/master/BLE-switcheroonie/BLE-switcheroonie.ino
https://github.com/T-vK/ESP32-BLE-Mouse
