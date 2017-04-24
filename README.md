# Projeto 2 - Robotica Computacional

## Filtro de Partículas e Probabilidade

### Por Alexandre Young e Sabrina Simão

### Prof. Fabio Miranda e Igor Montagner

Este projeto consiste na criação de um filtro de partículas para implementar em um robo Neato. O propósito é criar um programa que permita o robô se localizar no espaço, ao ser ligado e não ter informação de onde se encontra.

Isso é necessário, pois os robôs não costumam ter um gps do local em que se encontra, portanto precisa se "descobrir" geograficamente. Contudo, ele tem um mapa do lugar que vai percorrer, então sabe onde estão as paredes e obstáculos.

O robo possui mecanismos de localização como lasers, que conseguem detectar paredes e sua distância. Dessa forma, cria-se uma amostra aleatória de partículas, jogadas pelo mapa, e deve-se comparar o que as partículas estão recebendo com o que o robô em si está recebendo.

Se o robo tem o laser 3 detectando uma parede a 10 metros de distância, e uma partícula tem o mesmo laser detectando a mesma distância, isso é levado em consideração e essa partícula tem altas chances de estar na posição real do robo. Caso a leitura dela nao condissesse com a leitura do robo, isso implicaria que esta partícula NÃO representa a posição real do robo.

Resumindo, faz-se essa análise com todos os lasers do robô e todas as partículas. No final, obtém-se uma lista de maior para menor probabilidade. Quanto maior a probabilidade de uma particula, mais filhos ela terá.
Os filhos representam uma nova wave de iterações, quando recriamos todas as partículas levando em consideração a wave anterior. Dessa forma, as particulas menos prováveis vão ficando "inférteis e somem ao longo da evolução".

Para concatenar todo esse trabalho, é criado um arquivo .gif com todas as imagens de todas as iterações, mostrando bem como as partículas vão tendendo à posição que o robo está. (Ou seja, ele descobriu onde está no mundo real).

O trabalho foi feito em conjunto através da plataforma Jupyter Notebook. Todas as etapas foram elaboradas em um mesmo notebook. Para auxiliar o projeto feito no Jupyter, e deixá-lo mais legível, foi criado um módulo pessoal do grupo chamado SIMYOUNG que contém todas as funções elaboradas pela dupla e não triviais ao projeto.

Dentro desse módulo, encontra-se uma descrição minuciosa do que cada função elabora, e uma explicação para os valores de constantes usados (como desvio padrão, número de partículas, etc).

Existem poucos commits feitos pelo grupo, porém cada um representa uma parte importante do projeto concluída. Os primeiros 2 commits são mínimos. O terceiro commit foi quando a dupla conseguiu desenhar as partículas no mapa. No quarto commit, todas as 9 etapas referentes à atividade já estavam concluídas e o gif pronto. Os commits seguintes são menos relevantes, adicionando comentários e observações teóricas.