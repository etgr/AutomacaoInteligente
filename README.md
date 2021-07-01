# Automação Inteligente
Repositório contendo as atividades desenvolvidas ao longo da disciplina de Automação Inteligente, utilizando o MATLAB/V-REP para aplicações de algoritmos de inteligência artificial e simulações em robôs móveis.

![fig3](https://user-images.githubusercontent.com/22371807/124191170-c717a880-da99-11eb-93ea-8f595bca626f.PNG)


# Atividade 1 - Modelagem do Robô Móvel
Foi estudada a modelagem de um robô móvel com tração diferencial, aplicando métodos de Newton e Laplace. O modelo obtido foi simulado utilizando um diagrama no MATLAB/Simulink.

![fig1](https://user-images.githubusercontent.com/22371807/124190413-9daa4d00-da98-11eb-8cf3-1bbfe48d825c.PNG) ![fig2](https://user-images.githubusercontent.com/22371807/124190866-596b7c80-da99-11eb-8bfa-8b34fc5f9c26.PNG)

# Atividade 2 - Controle go-to-goal utilizando Algoritmo Genético

Foi desenvolvido um controlador PID para ajuste automático da trajetória do veículo no problema go-to-goal, onde o robô deve ir de um ponto a outro em linha reta. Para a sintonia do controlador, foram testados métodos de otimização convexa e algoritmos genéticos.

![DistTrajeto](https://user-images.githubusercontent.com/22371807/124191785-af8cef80-da9a-11eb-9166-ff43d3de0acb.png)
![DistOrient](https://user-images.githubusercontent.com/22371807/124191805-b4ea3a00-da9a-11eb-8554-e96f7bc501be.png)

# Atividade 3 - Controle em ambiente desconhecido utilizando Lógica Fuzzy

Foi desenvolvido um controlador de comportamentos baseado em lógica fuzzy para o robô móvel em um ambiente com obstáculos. O robô possuia sensores ultrassônicos na frente e laterais, que foram utilizados para detectar obstáculos. O controlador continha a lógica necessária para o robô explorar o ambiente desviando de obstáculos até chegar ao destino desejado.

![TesteFuzzy1](https://user-images.githubusercontent.com/22371807/124192435-a9e3d980-da9b-11eb-8e35-a1105ecffa7b.png)

![TesteFuzzy5](https://user-images.githubusercontent.com/22371807/124192441-ac463380-da9b-11eb-9bb0-efa7f2b31278.png)

# Atividade 4 - Projeto de um Sensor de Erro de Trajetória para um Robô Seguidor de Linha

Foi desenvolvido um arranjo de LEDs emissores e receptores infravermelhos, utilizados na determinação do erro de trajetória de um robô seguidor de linha.

![fig4](https://user-images.githubusercontent.com/22371807/124193482-43f85180-da9d-11eb-860d-c6924d662237.PNG)
![p3dx](https://user-images.githubusercontent.com/22371807/124192766-31314d00-da9c-11eb-9ae4-dff69b06c48b.PNG)


Alguns algoritmos de cálculo do erro disponíveis na literatura foram aplicados, cujos resultados foram comparados com os dados obtidos no simulador.
 
 ![variaveisErro](https://user-images.githubusercontent.com/22371807/124192991-81a8aa80-da9c-11eb-85d1-dbbb447c95f5.PNG)
 ![testeC](https://user-images.githubusercontent.com/22371807/124193015-8a00e580-da9c-11eb-8d84-561096b279f6.png)
 
# Atividades 5 e 6 - Projetos de Controladores de Erro de Trajetória Linear e Não-Linear para o Robô Seguidor de Linha
 
Nessas atividades foram aplicadas técnicas de sintonia de controladores proporcional, não-linear e fuzzy, disponíveis na literatura, para o controle do robô seguidor de linha, tendo como base erro de trajetória obtido com o sensor projetado na atividade 4.

![comparaPNLF](https://user-images.githubusercontent.com/22371807/124193563-6d18e200-da9d-11eb-9466-7233aafb1016.png)

 # Atividade 7 - Sensor de Erro de Trajetória Utilizando Redes Neurais Artificiais
 
Nessa atividade, uma rede neural foi treinada para calcular o erro de trajetória baseada nos dados dos sensores. O conjunto de dados foi obtido no simulador posicionando o robô de diversas formas, afim de obter diferentes valores dos sensores e erros de trajetória. O MATLAB foi utilizado para treinar uma rede feedforward com esses dados, e os resultados de erro de trajetória foram comparados com os algoritmos da atividade 4.

![estruturaRN](https://user-images.githubusercontent.com/22371807/124193997-2d062f00-da9e-11eb-882f-96ffa4639ace.PNG)
![desempRNgamma](https://user-images.githubusercontent.com/22371807/124194012-31324c80-da9e-11eb-8aee-945bacbfe00b.png)

![testeRN](https://user-images.githubusercontent.com/22371807/124194121-5fb02780-da9e-11eb-9d2d-463d86842d85.png)



