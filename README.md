# Embedded_Systems_Project_AVR
Old school project to create a PID controller for temperature control running a small DC motor fan using Atmel AVR XMEGA-B1 Explained board.

- Requirements:
    - Install and create a new project for the Atmel AVR board (atxmega128b1 recommended).
    - Import the .c and header files into your project.
    - LM19 temperature sensor.
    - Other materials explained further below.

- Description:
    *The PID-controller controls a temperature in an empty vessel.
    Temperature is controlled by changing rotational speed of the DC-motor (PWM output duty cycle). 
    The motor runs a fan that ventilates the vessel, which in turn is heated by a resistive load (during testing). 
    
    *A reference value for the temperature can be changed with bush buttons, USART, USB or capacitive buttons.
    The change of the reference value is handled by a common source (atomic operation). 
    Finally, the temperature value is read by a temperature sensor (LM19) connected to the AVR's internal AD-converter.
    

- State-machine diagram
    - The functionality of the setup:
    ![State Machine](/img/State_Machine.png)

- USART
    - Keyboard controls for the USART communication:
    ![USART](/img/USART.png)


- Hardware Setup:
    - (explanation, TODO)