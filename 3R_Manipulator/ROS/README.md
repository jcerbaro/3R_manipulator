Como Preparar o Ambiente de Trabalho para Trabalhar com o Manipulador 3R do LASER: Beckman Coulter ORCA Robotic Arm
-------------------------------------

HARDWARE
-------------------------------------

O manipulador possui 3 DoF dispostos de forma planar. Cada DoF é assosiado a um motor, além de um motor para o acionamento da garra. Os motortes suportam até 24V porém, para operação segura, sugere-se alimentação com 12V DC, a qual será regulada de 0V a 12V pela largura do PWM nos Arduinos. Há 2 Arduinos que controlam a estrutura, ambos acoplados em uma placa Monster Motor Shield que possuem duas ponte-H cada. Cada ponte-H controla o acionamento de um motor em ambos os sentidos.

Cada motor está acoplado a um encoder, o qual é alimentado com 5V provenientes do próprio Arduino. Internamente, há redutores e que alteram a velocidade relativa entre motor e encoder, definindo as relações de pulsos por grau de rotação do DoF. Para cada 1 grau de rotação, os encoders emitem a seguinte quantidade de pulsos digitais para o Arduino:
	Ombro: 196
	Cotovelo: 325
	Punho: 148

Cada encoder possui duas saídas que emitem o pulso de forma brevemente defasada (encoder de quadratura). Se o motor estiver rotacionando no sentido horário, o pulso da saída B é emitido brevemente antes da saída A. Vice-versa para o sentido anti-horário. Essa defasagem permite ao Arduino saber para que lado o motor está rotacionando.

Não há sensores fim-de-curso na estrutura, portanto o controle de parada dos motores é realizado pelo Arduino. No firmware padrão, se o motor está acionado porém os encoders pararam de emitir pulsos, isso significa que o motor está forçando a própria estrutura ou um obstáculo. Neste momento o Arduino manda o motor parar para evitar sobrecorrentes.

Não há sensores do tipo potenciômetro para indicar a posição do manipulador a qualquer instante. Para realizar o controle de posição de forma precisa, no firmware padrão dos Arduinos o manipulador é acionado para convergir a uma configuração conhecida toda a vez que é energizado. Essa é a posição inicial e todo o controle parte do pressuposto de que o controlador inicia nesta condição. Estes são os ângulos de cada junta que representam a pose "origem" do manipulador:
	Ombro: -40 graus
	Cotovelo: 145 graus
	Punho: -133 graus


COMUNICAÇÃO COM O ARDUINO
-------------------------------------

Nesta sessão apresenta-se o passo a passo para realizar a comunicação com múltiplos Arduinos com ROS pelo USB. Também menciona-se detalhes para a gravação de novos firmwares diferentes dos originais, caso seja necessário.

Para poder utilizar as bibliotecas ROS e estabelecer a comunicação serial para que o firmware do Arduino se comporte como um "nó ROS", é necessário instalar as bibliotecas e pacotes conforme descrito no tutorial oficial do site: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

Resumindo o tutorial, o que deve ser feito para ter um nó ROS nos Arduinos:

	1 - Baixe a IDE do Arduino em https://www.arduino.cc/en/Main/Software.

	2 - Para instalar a IDE, execute pelo terminal o arquivo ./install.sh que foi baixado.
			$ cd ~/Downloads/<arduino-XXX-linux64>/arduino-XXX/
			$ sudo ./install.sh

	3 - Instale o pacote rosserial para Arduinos, atentar para a <DISTRO> usada:
			$ sudo apt-get install ros-<DISTRO>-rosserial-arduino
			$ sudo apt-get install ros-<DISTRO>-rosserial

	4 - Vá para a pasta libraries dentro da sua sketchbook do Arduino (por padrão estará na Home)
			$ cd ~/Arduino/libraries/
			$ rm -rf ros_lib
  			$ rosrun rosserial_arduino make_libraries.py .
  														 ^ o ponto faz parte do comando!

Para poder gravar um novo firmware nos Arduinos, é necessário ter permissão de acesso aos dispositivos externos em portas USB. A forma mais eficiente resolver este problema é adicionar o usuário ao grupo "dialout". Portanto, basta inserir o comando:

			$ sudo usermod -a -G dialout $USER

Será necessário re-logar na máquina para a alteração ser reconhecida.

Antes de enviar o firmware para gravação e caso exista mais de um Arduino conectado via USB, lembre de verificar em qual porta USB o firmware está sendo gravado. Isso deve ser verificado na própria IDE do Arduino em "Tools > Port". Também deve ser verificado se versão da placa na IDE é a mesma que a física em "Tools > Board". As duas placas padrão do robô 3R são Arduino Mega 2560.

Em qualquer firmware que irá usar comunicação por ROS, a primeira linha deve conter "#include <ros.h>", antes das demais bibliotecas específicas de cada mensagem como "#include <custom_msg/set_angles.h>", por exemplo.

Depois que o firmware estiver gravado no Arduino, o nó pode ser iniciado pelo comando:

			$ rosrun rosserial_arduino serial_node.py /dev/XXX

Onde XXX deve ser o nome da porta USB em que o Arduino está conectado (será algo como "/ttyUSB0" ou "/ttyACM0"). Se nenhuma porta for especificada, será usada a "/ttyUSB0" por padrão. Uma maneira de ver o nome certo da porta é inserindo o comando "ls" dentro da sua pasta /dev, então removendo o cabo USB e inserindo "ls" novamente. A porta que sumir é a do Arduino. Usando o hub padrão do robô, o Arduino do ombro/cotovelo fica em /ttyUSB0 e punho/garra em /ttyUSB1

Na forma como o pacote rosserial foi projetado, só é possível executar 1 nó do Arduino por vez. Caso seja aberto um novo terminal e executado novamente o comando que utiliza o serial_node.py, o primeiro será abortado uma vez que se está querendo usar o mesmo arquivo Phyton em 2 nós diferentes. A solução mais simples para isso é criar um novo arquivo executável com outro nome.

Para habilitar a execução do segundo (ou do n-ésimo) nó de Arduino simultaneamente, navegue até a pasta do pacote:
	$ cd /opt/ros/<DISTRO>/share/rosserial_arduino

Faça uma cópia do arquivo "serial_node.py" e renomeie-o como desejar (exemplo: serial_node_2.py). Precisa executar como root porque estamos fora da home.

	$ sudo cp serial_node.py serial_node_2.py 

Abra a cópia criada e procure nas primeiras linhas o comando "rospy.init_node": o argumetno que estará entre aspas neste comando deve ser alterado para o nome do arquivo que você criou.

Finalizando, é necessário permitir que o novo arquivo seja executado. Para isso, dentro da pasta onde eles foram criados insira o comando:
	$ sudo chmod +x serial_node_2.py

Lembrando de usar o nome do(s) arquivo(s) que você criou. Também é necessário rodar novamente o comando "catkin_make" dentro da sua pasta "catkin_ws". Agora é possível iniciar quantos nós de Arduino simultaneamente forem necessários, um em cada terminal, associado ao seu próprio serial_node.py verificando qual a porta serial que a placa está conectada.

Dessa maneira, para utilizar os dois Arduinos como nós simultaneamente basta executar o comando:

	$ rosrun rosserial_arduino serial_node.py /dev/ttyUSB0

E em outro terminal:

	$ rosrun rosserial_arduino serial_node_2.py /dev/ttyUSB1

Possuindo o pacote padrão de controle do robô 3R, não é necessário fazer alterações nos nós do Arduino como descrito acima, uma vez que eles já estão na pasr /src.

Para acionar os Arduinos utilizando o pacote padrão, use os seguintes comandos em terminais separados (a porta USB já está configurada, então não precisa escrever ela no comando):

	$ rosrun lab_arm_control Arduino_1.py

	$ rosrun lab_arm_control Arduino_2.py

Para enviar os comandos, basta rodar um dos pacotes de controle, por exemplo o que permite o envio de ângulos a cada junta do robô "Publica_Angulos":

	$ rosrun lab_arm_control Publica_Angulos

Para lançar o nó desejado e simultaneamente os nós dos Arduinos e o roscore, sem precisar navegar entre terminais, basta executar o roslaunch, alterando "<ARQUIVO>" pelo nome do nó de controle que deseja usar:

	$ roslaunch lab_arm_control <ARQUIVO>.launch

Exemplo:

	$ roslaunch lab_arm_control Publica_Angulos.launch



------------------------------------
----- MENSAGEM ROS CUSTOMIZADA -----
------------------------------------

Possuindo o pacote "custom_msg", o qual foi criado para mensagens customizadas do LASER, basta incluí-lo na criação de novos pacotes e as mensagens poderão ser utiliizadas.

Para criar um arquivo desse tipo, novo, o arquivo CMakeLists.txt deverá ficar semelhante ao abaixo, onde "NODE" deve ser substituído pelo nó que será gerado, "CODE" deve ser substituído pelo nome do arquivo .cpp que gerará o executável e "PACOTE" deve ser substituído pelo nome do pacote criado. Para programação em Phyton, analisar diferenças de implementação.


				cmake_minimum_required(VERSION 2.8.3)
				project(PACOTE)

				find_package(catkin REQUIRED COMPONENTS
				  roscpp
				  custom_msg
				)

				include_directories(
				  ${catkin_INCLUDE_DIRS}
				)

				catkin_package(
				  CATKIN_DEPENDS
				)

				add_executable(NODE src/CODE.cpp)
				target_link_libraries(NODE ${catkin_LIBRARIES})
				add_dependencies(NODE ${catkin_EXPORTED_TARGETS})


O arquivo package.xml deverá ter a tag "<depend>custom_msg</depend>", lembrando que "depend" é um atalho para ambas "build_depend" e "exec_depend".

No arquivo .cpp, deve-se incluir a biblioteca de mensagens customizadas: #include "custom_msg/laser_msg.h"

Abaixo, um exemplo de publisher que publica um valor no tópico "publisher" para verificar o funcionamento através do comando "rostopic echo /publisher":

				#include "ros/ros.h"
				#include "custom_msg/laser_msg.h"

				int main(int argc, char **argv)
				{	
					ros::init(argc, argv, "pub");
					ros::NodeHandle n;

					ros::Publisher publisher = n.advertise<custom_msg::laser_msg>("publisher", 1000);
					ros::Rate looprate(1);

					int count = 0;

					while(ros::ok()){

						custom_msg::laser_msg msg;
						msg.ang_OMB = count;

						publisher.publish(msg);
						ros::spinOnce();
						looprate.sleep();
						count++;
					}
					return 0;
				}

O pacote padrão da custom_msg já está pronto. As diretrizes acima servem principalmente para guiar a criação de novos pacotes de mensagens customizadas. Depois de incluir o pacote dentro da sua catkin_ws, lembre de rodar o comando catkin_make.

Para o Arduino poder utilizar a mensagem customizada, deverá ser gerado o cabeçalho em C. Para isso, primeiramente deve-se deletar a pasta "ros_lib" que está  dentro de Arduino/libraries. Os comandos a seguir vão reconstruir todas as bibliotecas que relacionam ROS e Arduino, incluindo a biblioteca relacionada à mensagem customizada que foi criada (será gerado o header para usar no código). Caso você tenha alguma pasta pessoal dentro de libraries/ros_lib, faça o backup da mesma. Até o momento da escrita deste tutorial, o comando apenas funciona se você DELETAR A PASTA "ros_lib" para que a mesma seja gerada novamente. IMPORTANTE: este comando deve ser repetido toda vez que for feita alguma alteração na estrutura da mensagem customizada.
		
		$ cd ~/Arduino/libraries/
		$ rm -rf ros_lib
		$ rosrun rosserial_arduino make_libraries.py .
													 ^ o ponto faz parte do comando!

------------------------------------
----- FIRMWARE ARDUINOS PADRÃO -----
------------------------------------

É possível gravar qualquer firmware nos Arduinos acoplados ao manipulador, porém foram criados os firmwares considerados padrão para as funções comuns no LASER. Aqui, detalha-se o funcionamento destes algoritmos. Pormenores estão destacados em comentários inseridos no código-fonte. A única biblioteca externa que precisa ser instalada para compilar e alterar os códigos originais é a "AutoPID.h".

Ambos Arduinos iniciam rodando a função "reset" que coloca a estrutura no ponto de referência, recolhida, com a garra aberta.

DESCREVER O FUNCIONAMENTO DOS TÓPICOS.




FALTA FAZER
-----------------------
Calibrar a relação pulsos por grau.

Calibrar o ângulo real com o offset inicial.

Adicionar rotinas do I e D.

Testar a garra.