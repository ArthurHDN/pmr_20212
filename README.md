# TPSS - Planejamento de Movimento de Robôs - EEE935
## Repositório para os Trabalhos Práticos 2021/2
### Álvaro Rodrigues Araujo - 2018020034
### Arthur Henrique Dias Nunes - 2018020670

## Configurando o Ambiente
1. Certifique-se de usar um abiente ROS. A versão recomendada e utilizada neste repositório é [*Melodic*](http://wiki.ros.org/melodic) 
2. Certifique-se de ter [configurado uma _workspace_](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
3. Este repositório é um pacote ROS. Portanto, clone-o para a pasta src/ da sua workspace:

```bash
$ cd <my workspace>/src
$ git clone https://github.com/ArthurHDN/pmr_20212.git
```

4. Reconstrua sua workspace

```bash
$ cd <my workspace>
$ catkin_make
```

## TP1
### Tarefa 1: Tangent Bug

```bash
$ roslaunch pmr_20212 TP1_task1.launch
```

### Tarefa 2: Trajectory Tracking

```bash
$ roslaunch pmr_20212 TP1_task2.launch
```
<div style="text-align:center"><img src="https://github.com/ArthurHDN/pmr_20212/blob/main/media/gif_trajectory_tracking.gif"/></div>

### Tarefa 3: Potential Fields

```bash
$ roslaunch pmr_20212 TP1_task3.launch
```
<div style="text-align:center"><img src="https://github.com/ArthurHDN/pmr_20212/blob/main/media/gif_potential_fields.gif"/></div>

### Tarefa 4: Wave Front

```bash
$ roslaunch pmr_20212 TP1_task4.launch
```
<div style="text-align:center"><img src="https://github.com/ArthurHDN/pmr_20212/blob/main/media/gif_wave_front.gif"/></div>

## TP2
TBA
## TP Final Arthur
TBA
## TP Final Álvaro
TBA
## Referências
- Howie Choset et.al., Principles of Robot Motion: Theory, Algorithms, and Implementations
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). Robot modeling and control. Hoboken, NJ: John Wiley &
Sons
- [Arthur H. D. Nunes, The Turtles: o guia prático e introdutório de simulações em robótica com ROS](http://www.petee.cpdee.ufmg.br/minicursos_oficinas/#ros)
- [Como fazer um README bonitão](https://raullesteves.medium.com/github-como-fazer-um-readme-md-bonitão-c85c8f154f8)
