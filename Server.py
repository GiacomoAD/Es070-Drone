# -*- coding: utf-8 -*-
"""
Created on Sat Nov 14 19:16:18 2020

@author: GiacomoAD
"""

from threading import Thread
import socket
from queue import PriorityQueue
import curses

global svDisp
global svInput

cmd_q = PriorityQueue(maxsize = 5)

global droneOnline
droneOnline = 0

global recv_counter
recv_counter = 0

#########################################################################################################################
#                                                                                                                       #
#   Method Name: receive                                                                                                #
#                                                                                                                       #
#   Description: This method is used to receive text data from a socket connection, without decoding from bytes to text #
#                                                                                                                       #
#   Input parameters:   conn  -> Socket object that data will be received from                                          #
#                                                                                                                       #
#   Output parameters:  Returns the message in byte format                                                              #
#                                                                                                                       #
######################################################################################################################### 
def receive(conn):
     try:
         
         message_len = int(conn.recv(5).decode())   #Receiving first 5 bytes (incoming message length)
         return conn.recv(message_len)              #Returning the incoming message

     except socket.error as e:
         print(e)
         pass


#########################################################################################################################
#                                                                                                                       #
#   Method Name: sendD                                                                                                  #
#                                                                                                                       #
#   Description: This method is used to send text data over a socket connection, enconding it into bytes and            #
#                   marking the first 5 bytes with the message length.                                                  #
#                                                                                                                       #
#   Input parameters:   conn  -> Socket object that data will be sent to                                                #
#                       data  -> String that will be sent over the connection                                           #
#                                                                                                                       #
#   Output parameters:  n/a                                                                                             #
#                                                                                                                       #
#########################################################################################################################   
def sendD(conn, data):
    try:            
        data_b = str.encode(data)
        data_len = str(len(data_b))
        
        code = str.encode(data_len.rjust(5, '0'))   #Making a 5 byte sized string with '0's
        message = code + data_b                     #Concatenating length and message
        conn.sendall(message)                       #Sending resulting message
        
    except socket.error as e:
        print(e)
        pass
    

def authESP(conn):
    
    global svDisp
    
    sendD(conn, "#AEHDRONE")
    
    print("Autenticando...")
    svDisp.addstr(4,0,'Autenticando...')
    svDisp.refresh()
    key = receive(conn).decode()
    
    if(key == "OK"):
        return 1
    
    else:
        return 0
    



def threadedHandleDrone(conn):
    
    global droneOnline
    global recv_counter
    global svDisp
    global svInput
    
    command = "NULL"
    
    with conn:
        while(command != "#G"):
            try:
                command = cmd_q.get()[1]
                
            except:
                command = "NULL"
        
        print("ENVIANDO --->\t" + command)
        svDisp.addstr(7,0,'ENVIANDO COMANDO --->')
        svDisp.addstr(7,23, command)
        svDisp.refresh()
        svInput.refresh()
        
        sendD(conn, command)
        
        msg = receive(conn).decode(encoding="utf-8")
        print("RECEBIDO --->\t" + msg)
        svDisp.addstr(8,0,'MENSAGEM RECEBIDA --->')
        svDisp.addstr(8,24, str(msg))
        svDisp.refresh()
        svInput.refresh()

        while(droneOnline == 1):
            
            if(cmd_q.qsize() == 0):
                try:
                    cmd_q.put((3, "#K"))
                except:
                    pass
            else:
                pass
            
            print("AGUARDANDO...")
            msg = receive(conn).decode(encoding="utf-8")
            print("RECEBIDO --->\t" + msg)
            svDisp.addstr(8,0,'MENSAGEM RECEBIDA --->')
            svDisp.addstr(8,24, str(msg))
            svDisp.refresh()
            recv_counter += 1
            
            if(recv_counter == 5):
                command = cmd_q.get()[1]
                print("ENVIANDO --->\t" + command)
                svDisp.addstr(7,0,'ENVIANDO COMANDO --->')
                #svDisp.move(7,23)
                #vDisp.clrtoel()
                svDisp.addstr(7,23, command)
                svDisp.refresh()
                sendD(conn, command)
                recv_counter = 0

            
            
    

def main(stdscr):
        
    global droneOnline
    global svDisp
    global svInput

    
    #HOST = '192.168.43...' 
    HOST = '192.168.43.182'

    
    PORT = 25565        # Port to listen on (non-privileged ports are > 1023)
        

##%% SERVER CREATION AND CONNECTION SECTION

    #   Statement to ensure this will run only once and will not create infinite
    # instances recursively (Python error)
    if __name__ == '__main__':
                #OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
        curses.nocbreak()
        svDisp = curses.newwin(20,120,0,0)

        
        
        svInput = curses.newwin(4,120,21,0)
        curses.echo()
        svInput.keypad(True)
        
        svInput.addstr(0,0, "------------------------------------------------------------------------------------------------------------------------")
        svInput.addstr(1,0, "User Input >>")
        svInput.addstr(2,0, "------------------------------------------------------------------------------------------------------------------------")
        svInput.move(1,15)
        svInput.refresh()
        
        # Creating a socket object
        # AF_INET     -> IPv4 address
        # SOCK_STREAM -> TCP Connection
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        
        # Binding socket object to a specific host (IP) and network port
        try:
            s.bind((HOST, PORT))
        except socket.error as e:
            print(e)
        
        
        # Starting the server by listening to incoming connections
        s.listen()
        print('------SERVER STARTED------')
        svDisp.addstr(0,0,'------SERVER STARTED------')
        svDisp.refresh()
        

       
        
        
        while True:
            # When connection is found, store its socket object (conn) and address
            print('Listening to connections...\n')
            svDisp.addstr(2,0,'Listening to connections...')
            svDisp.refresh()
            try:
                conn, addr = s.accept()
                print(conn)
                print(addr)
                svDisp.addstr(3,0,"Cliente Conectado: " + str(addr))
                svDisp.refresh()
                
                if(authESP(conn)):
                    print('Autenticado!')
                    svDisp.addstr(5,0,'Autenticado!')
                    svDisp.refresh()
                    svInput.refresh()
                    client_thread = Thread(target=threadedHandleDrone, args=(conn,))
                    client_thread.daemon = True
                    client_thread.start()
                    droneOnline = 1
                    
                    while(droneOnline == 1):
                        
                        command = svInput.getstr().decode(encoding="utf-8")
                        cmd_q.put((1, command))
                        
                        svDisp.addstr(10,0, "ULTIMO COMANDO INSERIDO --->" + command)
                        svDisp.refresh()
                        
                        svInput.move(1,15)
                        svInput.refresh()
                        
                        
                    
                
                else:
                    print('ERRO na Autenticacao!\nEncerrando o programa!')
                    break
            
            except (KeyboardInterrupt, SystemExit, socket.error):
                s.close()
                print('Server Closed')
            
          
        s.close()
        print('Server closed')
    
        return


curses.wrapper(main)
    
main()

