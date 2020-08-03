# robotguia - Instrucciones
Código implementado para al simulación del robot guía usando visión por computador

Autor: Jose David Casadiego Lopez
e-mail: jd.casadiego10@uniandes.edu.co

Requisitos:
- Ubuntu 16.04 o mayor
- CoppeliaSim
- OpenPose 

Instrucciones:
1. Crear dentro del directorio del proyecto el directorio vacío ./camera/ o un directorio similar en donde se guardaran temporalmente las imagenes vistas por la cámara del simulador.
2. (Opcional/Recomendado) Asegurarse que dentro de la carpeta ./openpose_output/ existe por lo menos un archivo tipo .JSON. En modo de operación síncrono puede generar error si no existe un archivo previo a la ejecución. Se incluyen alguos archivos .JSON en el repositorio, pero en caso de ser necesario, se pueden generar ejecutando la simulación en modo de operación asíncrono.
3. Ejecutar el simulador CoppeliaSim y abrir la escena P3DX_sync_guide_test.ttt. NO iniciar la simulación desde el simulador.
4. En CoppeliaSim, asegurarse que Barra de menús -> Add-ons -> Blue Zero está activado.
5. En CoppeliaSim, revisar la ruta donde se guardan las imagenes en el child_script del robot P3DX.
6. En el programa cliente sync_guide_test.py, revisar las rutas correspondientes en la función readCameraFrame()
7. Desde una nueva terminal ejecutar el programa cliente sync_guide_test.py
