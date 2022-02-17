package frc.robot.subsystems;

import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.concurrent.TimeoutException;
import java.util.zip.CRC32;

import javax.print.DocFlavor.BYTE_ARRAY;
import javax.swing.text.StyledEditorKit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

class Datagram{
    int versionID;
    int packetLength;
    long frameID;
    long timestamp;
    int distance;
    int horiAngle;
    int vertAngle;
    long reserved;
    long checksum;
    boolean validPacket = false;

    Datagram(ByteBuffer byteBuffer)
    {
        byteBuffer.position(0);

        byte versionIDSigned = byteBuffer.get();
        versionID = Byte.toUnsignedInt(versionIDSigned);

        byte packetLengthSigned = byteBuffer.get();
        packetLength =  Byte.toUnsignedInt(packetLengthSigned);

        int frameIDSigned = byteBuffer.getInt();
        frameID = Integer.toUnsignedLong(frameIDSigned);

        int timestampSigned = byteBuffer.getInt();
        timestamp = Integer.toUnsignedLong(timestampSigned);

        short distanceSigned = byteBuffer.getShort();
        distance = Short.toUnsignedInt(distanceSigned);

        horiAngle = byteBuffer.getShort();
        vertAngle = byteBuffer.getShort();
        reserved = byteBuffer.getLong();
        checksum = byteBuffer.getInt();

        CRC32 crc = new CRC32();
        crc.update(byteBuffer);
        validPacket = (crc.getValue() == 0) ? true : false;
    }
}

public class SK22Vision extends SKSubsystemBase implements AutoCloseable {
    // destination ports are required, but source ports are optional
    final int odroidPort = 5005;
    // this will change depending on the network connected
    final String odroidIP = "";
    final int roborioPort = 5800;
    private String caughtException = "";
    ByteBuffer rDataBuffer = ByteBuffer.allocate(Constants.VisionConstants.UDP_PACKET_LENGTH).order(ByteOrder.LITTLE_ENDIAN);
    ByteBuffer packetBuffer = ByteBuffer.allocate(Constants.VisionConstants.UDP_PACKET_LENGTH).order(ByteOrder.LITTLE_ENDIAN);
    // not needed yet
    // byte[] sDataBuffer = new byte[1024];
    DatagramChannel sSocket;
    ByteArrayInputStream byteArrayInputStream;
    DataInputStream dataInputStream;

    // The packet containing the angle and distance values
    Datagram packetDatagram;

    public SK22Vision()
    {
        try {
            sSocket = DatagramChannel.open();
            sSocket.configureBlocking(false);
            sSocket.socket().bind(new InetSocketAddress(roborioPort));
            
        } catch (Exception e) {
            //TODO: handle exception
            System.out.println(e.toString());
        }

        

        // memory allocated for packets to be read on the computer
        // ideally the memory is not hardcoded, but dynamic based on the packet size

        // memory allocated for packets to be sent from the computer
        // ideally the memory is not hardcoded, but dynamic based on the packet size

    }
    
    // returns the boolean that represents whether or not an exception was caught
    public String caughtException(String exceptionMessage)
    {
        return caughtException;
    }

    // Receive the latest available UDP packet if any is available. Return
    // true if a packet has been read, false if none is available.
    //
    // This method discards all but the latest received packet on the 
    // socket.
    public boolean getPacket(DatagramChannel sSocket) 
    {
        boolean receivedOne = false;
        SocketAddress SockRet;
        int ReadCount = 0;

        do
        {
            rDataBuffer.position(0);
            ReadCount++;

            try
            {
                // get the packet at the port defined by sSocket
                SockRet = sSocket.receive(rDataBuffer);
                if (SockRet != null)
                {
                    // We got a packet! Save it into a local buffer
                    // for later.
                    receivedOne = true;
                    packetBuffer = rDataBuffer.duplicate();
                    packetBuffer.order(ByteOrder.LITTLE_ENDIAN);
                }
            }
            catch (Exception e) 
            {
                caughtException = e.toString();
                System.out.println("Socket Excetion: " + caughtException);
                SockRet = null;
            }
        } while(SockRet != null);

        return receivedOne;
    }

    public void sendPacket(byte[] sDataBuffer, DatagramSocket sSocket, String message)
    {
        try 
        {
            // converts string data to byte data and stores this in the Data Buffer to be sent
            sDataBuffer = message.getBytes();
            // takes byte data stored in sDataBuffer and sends it to the odroid's IP address at the specified port on the odroid
            DatagramPacket outputPacket = new DatagramPacket(sDataBuffer, sDataBuffer.length, InetAddress.getByName(odroidIP), odroidPort);
            sSocket.send(outputPacket); 
        }
        catch (IOException i)
        {
            System.out.println("IO Exception" + i.toString());
        }
    }
    
    @Override
    public void periodic()
    {
        boolean packetRecieved = getPacket(sSocket);
        // check if packet is valid
        if(packetRecieved)
        {
            packetDatagram = new Datagram(packetBuffer);
            if (packetDatagram.validPacket)
            {
                SmartDashboard.putNumber("Distance",            packetDatagram.distance);
                SmartDashboard.putNumber("Horizontal Angle",    packetDatagram.horiAngle);
                SmartDashboard.putNumber("Vertical Angle",      packetDatagram.vertAngle);
            }
        }
    }   

    
    @Override
    public void initializeTestMode()
    {

    }

    @Override
    public void testModePeriodic()
    {

    }

    @Override
    public void enterTestMode()
    {

    }

    @Override
    public void close() throws Exception{

    }


}
