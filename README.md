# Mechatronics Final Project

Development of a microcontroller based controller for a 5 DOF didatic manipulator robot and gripper.

[FINAL PAPER](https://drive.google.com/file/d/1IV7_ScXzZWTYZx_DazngR-hfs36UWTP1/view?usp=drive_link)

## Compilação

Primeiro clone o ropositório:

    git clone https://github.com/LabRobotica/OpenServer.git

Agora entre na pasta clonada, crie uma pasta chamada "build" e entre dentro dela:

    cd OpenServer && mkdir build && cd build

Por fim crie o make usando `cmake`, e compile usando o make:

    clear && cmake .. && clear && make 

### Execução

Ainda dentro da pasta "build", execute usando privilegios administrativos com o comando:

    clear && sudo ./OpenServer
