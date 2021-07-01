# Colibrino 
Esse é um projeto open-source de uma versão "Faça você mesmo" do nosso mouse de cabeça o Colibri! O objetivo principal dele é promover acessibilidade para pessoas com deficiências físicas do como tetraplegia, artrogripose, amputações e parilisia cerebral. 
(imagem de cabeçalho)

# Índice 

- [Colibrino](#colibrino)
- [Índice](#índice)
- [Licença de Uso](#licença-de-uso)
- [Materiais:](#materiais)
  - [Versão com fio](#versão-com-fio)
  - [Versão Bluetooth (Desenvolvimento)](#versão-bluetooth-desenvolvimento)
- [Montagem:](#montagem)
  - [Diagrama Esquemático](#diagrama-esquemático)
  - [Instruções de Montagem](#instruções-de-montagem)
- [Modo de Uso](#modo-de-uso)
- [Plano de Voo 🐦](#plano-de-voo-)
- [Materiais de referência:](#materiais-de-referência)

# Licença de Uso
Este programa é um software livre. Você pode redistribuí-lo e/ou
modificá-lo sob os termos da Licença Pública Geral GNU como publicada
pela Free Software Foundation; na versão 3 da Licença, ou
(a seu critério) qualquer versão posterior.

Este programa é distribuído na esperança de que possa ser útil,
mas SEM NENHUMA GARANTIA; sem uma garantia implícita de ADEQUAÇÃO
a qualquer MERCADO ou APLICAÇÃO EM PARTICULAR. Veja a
[Licença Pública Geral GNU](https://github.com/tix-life/Colibrino/blob/master/LICENSE) para mais detalhes.



# Materiais:
O Colibrino possui duas versões:
## Versão com fio

|Nome|Especificação|Sugestão de compra|
|---|---|---|
|Arduino Leonardo ou Micro Pro|Processador AT32U4|[Bau da Eletônica](https://www.baudaeletronica.com.br/placa-micro-r3.html)|
|MPU6050|Acelerômetro e Giroscópio||
|TCRT 5000|Sensor Infravermelho||
|Cabo com 6 vias|Ligação entre o Arduino e os Sensores||
|Placa padrão ou protoboard|Local de Montagem||
|Leds|Indicador de piscada e de funcionamento||
|Resistor|Controlador da corrente nos componentes||
|Buzzer|Indicador Sonoro da Piscada||
|Cabo USB|Conexão entre computador e Arduíno||
|Armação de Óculos|Suporte para os sensores||

## Versão Bluetooth (Desenvolvimento)

Obs: Será necessário todos os componentes da versão de fio juntamente com os da tabela a seguir.
|Nome|Especificação|Sugestão de compra|
|---|---|---|
|ESP32|Processador Bluetooth||
|Bateria 3.7v|Fonte de Energia||
|Carregador de bateria|Carregador da fonte de energia||


# Montagem:
A montagem descrita e representada a seguir é da versão com fio do Colibrino. Em breve iremos apresentar a versão sem fio. 
## Diagrama Esquemático
(Esquemático)
## Instruções de Montagem
1. Conecte o Arduíno, Buzzer, Leds e Resistores no Protoboard. 
2. Para os sensores MPU6050 e TCRT 5000 é necessário solda-los com o fio de 6 vias.
3. Realize as coneções mostradas no diagrama acima.
4. Conecte o Arduíno ao PC usando o cabo USB.
5. Instale o código utilizando o ambiente de desenvolvimento do Arduino devidamente configurado para a placa que está sendo usada.
6. Fixe o sensor MPU6050 em alguma armação de óculos.
7. Use e aproveite. :D

(talvez montar um painel com imagens)


# Modo de Uso
Com o colibrino devidamente montado para usá-lo basta mexer a cabeça com o oculos que está instalado o sensor. Contudo, segue abaixo uma lista de orientações.

1. O Sensor MPU6050 deve estar colocado no óculos que vai ser usado, porém o TCRT deve estar conectado somente ao fio permitindo o ajuste.
2. Para melhor detecção da piscadela é necessário mexer o fio que o sensor TCRT está conectado para mais próximo ao olho.

(explicar como que usa o colibrino)

# Plano de Voo 🐦
Existe inumeras funções que podem ser adcionadas ao Colibrino e algumas que estão no plano de voo da nossa equipe são:
* Scroll ao inclinar a cabeça para os lados
* Funções com gestos da cabeça

Estamos abertos a sugestões e feedbacks a todos :D 
Agradecemos a leitura <3

# Materiais de referência:
* https://www.youtube.com/watch?v=4sIkW7wogrE
* https://github.com/witnessmenow/arduino-switcheroonie/blob/master/BLE-switcheroonie/BLE-switcheroonie.ino
* https://github.com/T-vK/ESP32-BLE-Mouse
