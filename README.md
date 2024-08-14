<h1 align="center">Iniciacao-Cientifica</h1>
<hr>

<p align="center" height=200px>
<img loading="lazy" src="http://img.shields.io/static/v1?label=STATUS&message=EM%20DESENVOLVIMENTO&color=GREEN&style=for-the-badge">
</p>


### Topicos

- [Visao Computacional](#visao-computacional)

- [Arduino](#arduino)

- [Prototipo](#prototipo)

  
## Visao Computacional
neste topico foram estudados e implementados formas de se analisar e categorizar imagens, utilizando das bibliotecas Yolo, PyTorch, COCO e darknet; Fazendo a integração de todas estas para atingir um resultado satisfatorio
### resultados
  foi possivel utilizar webcam e captura de tela para fazer a identificação dos itens desejados e filtra-los, e tambem criar grid's para delimitação de area de ativação, fazendo assim com que seja possivel ativar funçoes expecificas apenas quando um objeto selecionado estiver na area escolhida

  <p align="center" height=200px>
  <img loading="lazy" src="https://github.com/LorenzoBertozzi/Iniciacao-Cientifica/blob/main/motion/capturas%20de%20tela/Captura%20de%20tela%202024-06-18%20105851.png">
  </p>

## Arduino
neste topico desenvolvemos a parte de cirquito eletronico que controla-ra os motores e sensores fixos de nosso prototipo, utilizando um arduino uno, ponte-H L928N, 2 motores com encoders e caixa de redução 6v de 100RPM, 2 conjuntos de baterias e um sensor ultrasonico.
### resultados
  foi posivel a partis da implemetação do circuito fazer com que o robo aja de forma semi independente (como um robo aspirados de pó), em sua saida Serial podemos obter informações deste, como rpm, Km/h, distancia do obj mais proximo e distancia percorrida, e tambem temos as funcoes de deslocamento, como mover o prototipo para um posição especifica a partir de (0,0), setar sua velocidade retilinea e valicidade linear e angular

  <p align="center" height=200px>
  <img loading="lazy" src="https://github.com/LorenzoBertozzi/Iniciacao-Cientifica/blob/main/Diagrama-Do-Circuito.png">
  </p>

## Prototipo
Neste topico imprimimos suportes 3d para suportar de forma provisoria nosso circuito e componentes eletronicos
### resultados 
  foi possivel imprimir placas de PLA para encaixe e suporte dos componentes, no entando o modelo atual nao esta em sua versao final, ja que deveremos ter um prototipo autonomo OutDoor

  <p align="center" height=200px>
  <img loading="lazy" src="https://github.com/LorenzoBertozzi/Iniciacao-Cientifica/blob/main/prototipo/capturas%20do%20projeto/Captura%20de%20tela%202024-06-04%20152506.png?raw=true">
  </p>
