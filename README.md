# RASCUNHO DO READ ME. FALTA COLOCAR AS PREMIACOES E ATUALIZAR COM MIDIAS E INFORMACOES NOVAS
# CoRA 2025 (Competição de Robôs Autônomos) - Advanced Line Follower

![CoRA2025 Banner](http://cora.cpdee.ufmg.br/img/imgs/banner_cora24.png)
MUDAR COM A FOTO DE 2025

This is a repository containing all the files utilised on the development and tests of our autonomous line follower robot.
The competition challengeS participants to prototype, build, program and test the robot from scratch, with the help of tools such as Arduino (and languages alike), and CAD and Simulation softwares.

## File Structure
- `data/`: All the relevant data acquired during testing and programs to do so
- `miscellaneous`: All the files regarding the building of the robot and circuitry
- `CoRA2024.ino`: Main file in Arduino to compile/run the code and test the robot
- `motores.cpp`: Specific script to implement the motorised functions of the robot
- `motores.h`: Library referring to `motores.cpp` and its functions
- `desafios.cpp`: Specific scripts to tackle the track's challenges with the robot 
- `desafios.h`: Library referring to `desafios.cpp` and its functions

## Technologies used
### Programming Languages
- [Arduino / C++](https://www.arduino.cc/): Main Framework
- [C](https://learn.microsoft.com/pt-br/cpp/c-language/): Libraries and other implementations
- [Python](https://python.org): Data Analysis
- [MATLAB](https://www.mathworks.com/products/matlab.html): Data Plotting and Analysis

### Hardware
<a href="miscellaneous/CoRA2024_ElectronicCircuit.png">
  <img src="miscellaneous/CoRA2024_ElectronicCircuit.png" alt="Electronic Circuit" width="500">
</a>
<p><b>This is a schematic of the electronic circuitry used.</b></p>

- [2x Yellow DC Motors](https://i0.wp.com/myduino.com/wp-content/uploads/2023/09/2-40.jpg?w=600&ssl=1) - **Specs:** 3V-12VDC; 1:48 gear ratio; 800g/cm max. @ 3VDC; 70mA (250mA max. @ 3VDC)
- [2x Yellow DC Motor Wheels](https://kitsguru.com/cdn/shop/products/tracked-wheel-for-bo-motor-yellow-26mm-width_2048x.jpg?v=1642829900)
- [1x Swivel Caster Wheel](https://www.institutodigital.com.br/wp-content/uploads/2020/10/rodizio-giratorio-25mm-2.png)
- [2x 18650 Lithium-ion battery cells](https://electrostoreshop.com/wp-content/uploads/2024/04/3-7v-2000mah-18650-rechargeable-li-ion-lithium-battery.jpg) - **Specs:** 3.7VDC; 2000mAh
- [1x Arduino Nano](https://store.arduino.cc/products/arduino-nano)
- [1x Breadboard and Jumper Cables](https://images.squarespace-cdn.com/content/v1/556646a4e4b0bda793faf918/1595041751201-D74U6ZYRW93A6ZW6YYMK/BRBRD830J+Breadboard+MB102+w.+65+Jumper+cables.jpg)
- [1x L293D CI Motor Driver](https://www.ti.com/lit/ds/symlink/l293.pdf?HQS=dis-mous-null-mousermode-dsf-pf-null-wwe&ts=1723753701665)
- [7x TCRT5000 Infrared Sensors](https://www.haoyuelectronics.com/Attachment/TCRT5000/tcrt5000.pdf)

## Media and Miscellaneous
<a href="https://www.youtube.com/watch?v=5h6ZvlgcDps">
  <img src="https://img.youtube.com/vi/5h6ZvlgcDps/0.jpg" alt="Participation in the Autonomous Robot Competition at UFMG (CoRA) 2024" width="500">
</a>
<p>This video showcases our participation in the Autonomous Robot Competition at UFMG (CoRA) 2025, and our performance in the Final round.</p>
MUDAR COM O VIDEO NOVO DE 2025

<a href="miscellaneous/CoRA2024_LaserCutChassis.png">
  <img src="miscellaneous/CoRA2024_LaserCutChassis.png" alt="Laser Cut Chassis" width="500">
</a>
<p>This is the vector of the chassis used to assemble the parts on, and thus the whole robot.</p>

<a href="miscellaneous/CoRA2024_SharkFrame3DPrinting.png">
  <img src="miscellaneous/CoRA2024_SharkFrame3DPrinting.png" alt="Shark Frame 3D Printing" width="500">
</a>
<p>This is the representation of our decorative frame for the robot.</p>

## Developers

<table>
  <tr>
    <td align="center" style="max-width: 200px;">
      <a href="https://github.com/lemosslucas" target="_blank">
        <img src="https://github.com/lemosslucas.png" width="180" />
      </a>
      <br/>
      <a href="https://github.com/lemosslucas"><strong>Lucas Lemos Ricaldoni</strong></a><br/>
      <span>Team Lead & Lead Programmer</span>
    </td>
    <td align="center" style="max-width: 200px;">
      <a href="https://github.com/mateusdcp13"> 
        <img src="https://github.com/mateusdcp13.png" width="180">
      </a>
      <br/>
      <a href="https://github.com/mateusdcp13"><strong>Mateus de Carvalho Pedrosa</strong></a><br/>
      <span>Hardware Lead & Prototyping Specialist</span>
    </td>
    <td align="center" style="max-width: 200px;">
      <a href="https://github.com/PauloMendesPVRM"> 
        <img src="https://github.com/PauloMendesPVRM.png" width="180">
      </a>
      <br/>
      <a href="https://github.com/PauloMendesPVRM"><strong>Paulo Vasconcelos Mendes</strong></a><br/>
      <span>Lead Mechanical Designer & Support Prototyper</span>
    </td>
  </tr>
</table>

### Name of the team: "Tartarugas Linhas"

## License
This project is licensed under the MIT License. See the file [LICENSE](LICENSE) for details.
