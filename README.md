# SEGUIDOR DE LINHA - PROJETO DE BIXO

Repositório de suporte para projeto desenvolvido por alunos de graduação do núcleo de robótica aérea (NRA) do grupo SEMEAR. 

Neste projeto desenvolveu-se um modelo de seguidor de linha para percorrer uma trajetória pré-estabelecida, que contém diversos desafios envolvendo as áreas de navegação e visão computacional. A imagem ilustrativa da trajetória está no arquivo pista.jpg.

# Trajetória proposta:
Inicialmente, o seguidor de linha deve começar seu percurso na faixa start e seguir em direção ao trecho circular de modo que encontre o bloco azul no cruzamento. Ao chegar no cruzamento o seguidor deve, na primeira volta, fazer uma curva para a esquerda e seguir o percurso até que encontre o bloco vermelho, onde deverá parar a 20 centímetros de distância do bloco por 5 segundos. Na sequência, o seguidor deve voltar a mover-se de modo a repertir o percurso inicial até que encontre o bloco azul novamente, onde deverá, por sua vez, fazer uma curva para a direita e seguir o percurso até que chegue na faixa finish, onde cessará o movimento e completará a trajetória. 

# Desenvolvimento do projeto:
O projeto foi desenvolvido no simulador CoppeliaSim, sendo que a linguagem de programação utilizada inicialmente foi Lua. Posteriormente, ocorreu uma transição para a linguagem C++ utilizando o Remote API no ambiente do Ubuntu. Para possibilitar a navegação do seguidor pela trajetória indicada pela linha preta implementou-se o controle PID. Além disso, utilizou-se a biblioteca OpenCV para possibilitar o processamento das imagens captadas pela câmera do seguidor e, posterior  detecção das cores dos blocos. 

# Documentação do projeto: 
Com o intuito de documentar a evolução do projeto diversos arquivos foram incluídos neste repositório e suas funções serão explicadas a seguir:

Script_SeguidorTeste.lua: Documentação do código desenvolvimento no início do projeto para um seguidor de linha teste com intuito de facilitar o aprendizado da linguagem Lua e familiarização com o ambiente do simulador CoppeliaSim.

Script_SeguidorFinalizado_LinguagemLua: Código em linguagem Lua desenvolvido para um modelo de CAD parcial. É importante destacar que não foi implementada a biblioteca OpenCV neste código.

main.cpp: Script do código finalizado em linguagem C++, com parâmetros ajustados para o CAD finalizado do seguidor de linha. Neste código, tem-se a implementação da biblioteca OpenCV e do controle PID.

Script_ControlePID.cpp: Script separado do algoritmo para implementação do controle PID.

Script_VisãoComputacional.cpp: Script separado do algoritmo para realização das tarefas da visão computacional.
