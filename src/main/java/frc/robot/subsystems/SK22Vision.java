package frc.robot.subsystems;

import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;
import java.util.Optional;
import java.util.zip.CRC32;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

/**
 * The Datagram Class defines and unpacks the contents of the packets sent by the
 * Odroid-XU4
 */
class Datagram
{
    int     versionID;
    int     packetLength;
    long    frameID;
    long    timestamp;
    Optional<Double>  distance = Optional.empty();
    int     adjVal;
    Optional<Float>   horiAngle = Optional.empty();
    Optional<Float>   vertAngle = Optional.empty();
    
    long    reserved;
    long    checksum;
    boolean validPacket = false;

    Datagram(ByteBuffer byteBuffer)
    {
        byteBuffer.position(0);

        byte versionIDSigned = byteBuffer.get();
        versionID = Byte.toUnsignedInt(versionIDSigned);

        byte packetLengthSigned = byteBuffer.get();
        packetLength = Byte.toUnsignedInt(packetLengthSigned);

        int frameIDSigned = byteBuffer.getInt();
        frameID = Integer.toUnsignedLong(frameIDSigned);

        int timestampSigned = byteBuffer.getInt();
        timestamp = Integer.toUnsignedLong(timestampSigned);

        short distanceSigned = byteBuffer.getShort();
        int distanceInt = Short.toUnsignedInt(distanceSigned);

        byte adjValSigned = byteBuffer.get();
        adjVal = Byte.toUnsignedInt(adjValSigned);

        short shortHoriAngle = byteBuffer.getShort();
        

        short shortVertAngle = byteBuffer.getShort();

        reserved = byteBuffer.getLong();
        checksum = byteBuffer.getInt();

        CRC32 crc = new CRC32();
        crc.update(byteBuffer);
        if (crc.getValue() == 0 || distanceInt == 0)
        {
            distance = Optional.of(distanceInt / (Math.pow(10, adjVal)));
            horiAngle = Optional.of((float) shortHoriAngle / 100);
            vertAngle = Optional.of((float) shortVertAngle / 100);
        }
    }
}

/**
 * This class retrieves packets sent by the Odroid-XU4 at port 5800 and unpacks the data
 * using the Datagram class above, within the periodic
 */
public class SK22Vision extends SKSubsystemBase implements AutoCloseable
{
    // destination ports are required, but source ports are optional
    final int odroidPort = VisionConstants.ODROID_PORT;

    final int roborioPort = VisionConstants.ROBORIO_PORT;

    ByteBuffer rDataBuffer  =
            ByteBuffer.allocate(VisionConstants.UDP_PACKET_LENGTH).order(ByteOrder.LITTLE_ENDIAN);
    ByteBuffer packetBuffer =
            ByteBuffer.allocate(VisionConstants.UDP_PACKET_LENGTH).order(ByteOrder.LITTLE_ENDIAN);

    DatagramChannel      sSocket;
    ByteArrayInputStream byteArrayInputStream;
    DataInputStream      dataInputStream;

    // The packet containing the angle and distance values
    Datagram packetDatagram;

    /**
     * This constructor initializes the port 5800 on the roborio for reading UDP packets.
     * Disables blocking so it can be used in periodic
     */
    public SK22Vision()
    {
        try
        {
            sSocket = DatagramChannel.open();
            sSocket.configureBlocking(false);
            sSocket.socket().bind(new InetSocketAddress(roborioPort));
        }

        catch (Exception e)
        {
            //TODO: handle exception
            DriverStation.reportError("Failed to Initialize Socket in Constructor",
                e.getStackTrace());
        }

        // memory allocated for packets to be read on the computer
        // ideally the memory is not hardcoded, but dynamic based on the packet size

        // memory allocated for packets to be sent from the computer
        // ideally the memory is not hardcoded, but dynamic based on the packet size

    }

    /**
     * Receive the latest available UDP packet if any is available. Return true if a
     * packet has been read, false if none is available.
     *
     * This method discards all but the latest received packet on the socket.
     * 
     * @param sSocket
     *            A non-blocking DatagramChannel object that is used to initialize the
     *            packet receiving port on the roborio
     * 
     * @return True/False if a packet was received at the DatagramChannel on the specified
     *         port
     */
    public boolean getPacket(DatagramChannel sSocket)
    {
        boolean receivedOne = false;
        SocketAddress sockRet;

        do
        {
            rDataBuffer.position(0);

            try
            {
                // get the packet at the port defined by sSocket
                sockRet = sSocket.receive(rDataBuffer);
                if (sockRet != null)
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
                DriverStation.reportError("Packet Lost", e.getStackTrace());
                sockRet = null;
            }
        }
        while (sockRet != null);

        return receivedOne;
    }

    @Override
    public void periodic()
    {
        boolean packetRecieved = getPacket(sSocket);
        // check if packet is valid
        if (packetRecieved)
        {
            packetDatagram = new Datagram(packetBuffer);
            if (packetDatagram.validPacket)
            {
                SmartDashboard.putNumber("Distance", packetDatagram.distance.get());
                SmartDashboard.putNumber("Horizontal Angle", packetDatagram.horiAngle.get());
                SmartDashboard.putNumber("Vertical Angle", packetDatagram.vertAngle.get());

                SmartDashboard.putBoolean("Target Acquired",
                    isTargetAcquired(packetDatagram.horiAngle));
            }
        }
    }

    /**
     * Gets the horizontal angle of the robot relative to the center of the hub.
     * 
     * @return The horizontal angle in degrees
     */
    // TODO: Correct this has for correct documentation and
    // make sure it is returning the correct value.
    public Optional<Float> getHorizontalAngle()
    {
        return packetDatagram.horiAngle;
    }

    /**
     * Gets the vertical angle of the robot relative to the top of the hub
     * 
     * @return The vertical angle in degrees
     */
    // TODO: Correct this has correct documentation and
    // make sure it is returning the correct value.
    public Optional<Float> getVerticalAngle()
    {
        return packetDatagram.vertAngle;
    }

    /**
     * Gets the distance of the robot relative to the center of the hub.
     * 
     * @return The distance in meters
     */
    // TODO: Correct this has correct documentation and
    // make sure it is returning the correct value.
    public Optional<Double> getDistance()
    {
        return packetDatagram.distance;
    }

    /**
     * Checks if the angle is within 0º ± tolerance
     * 
     * @param horiAngle
     *            The measured angle in degrees
     * @return Whether the angle is withing the tolerance
     */
    public boolean isTargetAcquired(Optional<Float> horiAngle)
    {
        return
        // Above 0º - Tolerance 
        (horiAngle.get() >= -VisionConstants.TARGET_ACQUIRED_TOLERANCE
            // Below 0º + Tolerance
            && horiAngle.get() <= VisionConstants.TARGET_ACQUIRED_TOLERANCE);
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
    public void close() throws Exception
    {

    }

}
