import time
from pySerialTransfer import pySerialTransfer as txfer

class ArduinoHandler:
    """
    Simple class to send/receive commands to and from Arduino.
    """

    def __init__(self, port, printer):
        self.port = port
        self.baudrate = 500000
        self.timeout = 0.05
        self._link = None
        self.printer = printer
        self._open()


    def _open(self):
        self.close()
        try:
            self._link = txfer.SerialTransfer(self.port, baud=self.baudrate)
            self._link.open()     
            time.sleep(2.0)  # give Arduino time to reset
            self.printer(f"Connected to Arduino on port {self.port}")


        except Exception as e:
            self.printer(f"Could not connect to Arduino: {e}")
            self._link = None




    def send(self, Bx, By, Bz, alpha, gamma, freq, psi, gradient, equal_field, acoustic):
        """
        Send actuation command packet (binary encoded).
        All arguments should be float or int.
        """
     
        if self._link is not None:
          
            try:
                idx = 0
                idx = self._link.tx_obj(float(Bx), start_pos=idx)
                idx = self._link.tx_obj(float(By), start_pos=idx)
                idx = self._link.tx_obj(float(Bz), start_pos=idx)
                idx = self._link.tx_obj(float(alpha), start_pos=idx)
                idx = self._link.tx_obj(float(gamma), start_pos=idx)
                idx = self._link.tx_obj(float(freq), start_pos=idx)
                idx = self._link.tx_obj(float(psi), start_pos=idx)
                idx = self._link.tx_obj(float(gradient), start_pos=idx)
                idx = self._link.tx_obj(float(equal_field), start_pos=idx)
                idx = self._link.tx_obj(float(acoustic), start_pos=idx)

                self._link.send(idx)
              
            except Exception as e:
                self.printer(f"Error sending data: {e}")

    def receive(self):

        if self._link is None:
            return None


        if self._link.available(): 
            idx = 0
            currents = []

            for _ in range(6):
                val = self._link.rx_obj(float, start_pos=idx)
                idx += 4
                currents.append(val)

            
            return currents
        
        
        return None




    def close(self):
        """Close the serial connection."""
        if self._link is not None:
            try:
                self._link.close()
            except Exception as e:
                self.printer(f"Error closing connection: {e}")
            finally:
                self._link = None
                
        


# Example usage:
if __name__ == "__main__":
    def printer(msg):
        print(msg)
    arduino = ArduinoHandler(port="COM5", printer=printer)
    time.sleep(2)
    print("open arduino srial monitor so you can see data being received")


    #arduino.send(B, B, B, B, B, B, B, B, B, B)
    arduino.send(1, 0, 0, 0, 0, 0, 0, 0, 0, 0)
   
    time.sleep(2)
    arduino.send(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    currents = arduino.receive()
    if currents:
        print("Currents received:", currents)
    print("Zero command sent")
    arduino.close()
 