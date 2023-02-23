# Alarms description and analysis: 

0x00: 'reset occurred'  **Czy to działa?**  
0x01: 'undefined instruction'  **NIE**   
0x02: 'file system error'  **TAK**    
0x03: 'communications error between MCU and FPGA'  **TAK**    
0x04: 'angle sensor error'  **TAK**       

0x10: 'plan: pose is abnormal' - **CZY WYSTĘPUJE?**    
0x11: 'plan: pose is out of workspace' - **CZY WYSTĘPUJE?**    
0x12: 'plan: joint limit' - **CZY WYSTĘPUJE?**  
0x13: 'plan: repetitive points' - **Sprawdzanie, czy punkty na łuku nie są tożsame**      
0x14: 'plan: arc input parameter' - **NIE ISTNIEJE wg dokumentacji**    
0x15: 'plan: jump parameter' - **Sprawdzanie warunku, czy high > 0**  

0x20: 'motion: kinematic singularity' - **CZY WYSTĘPUJE?**  
0x21: 'motion: out of workspace'  - **NISKOPOZIOMOWA OBSŁUGA PRZEZ SERWER AKCJI**   
0x22: 'motion: inverse limit'  - **NISKOPOZIOMOWA OBSŁUGA PRZEZ SERWER AKCJI**     

0x30: 'axis 1 overspeed'  **DYDAKTYCZNIE TAK**  
0x31: 'axis 2 overspeed'  **DYDAKTYCZNIE TAK**   
0x32: 'axis 3 overspeed'  **DYDAKTYCZNIE TAK**   
0x33: 'axis 4 overspeed'  **DYDAKTYCZNIE TAK**   

0x40: 'axis 1 positive limit'  **TAK**       
0x41: 'axis 1 negative limit'  **TAK**    
0x42: 'axis 2 positive limit'  **TAK**     
0x43: 'axis 2 negative limit'  **TAK**      
0x44: 'axis 3 positive limit'  **TAK**      
0x45: 'axis 3 negative limit'  **TAK**      
0x46: 'axis 4 positive limit'  **TAK**      
0x47: 'axis 4 negative limit'  **TAK**      
0x48: 'Parallelogram Positive Limitation Alarm'  **TAK**    
0x49: 'Parallelogram Negative Limitation Alarm'  **TAK**    

0x50: 'axis 1 lost steps'  **TAK**  
0x51: 'axis 2 lost steps'  **TAK**  
0x52: 'axis 3 lost steps'  **TAK**  
0x53: 'axis 4 lost steps'  **TAK**  