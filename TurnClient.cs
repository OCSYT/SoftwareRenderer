using System.Net;
using System.Net.Sockets;
using System.Text;

namespace SoftwareRenderer
{
    public class TurnClient
    {
        private const Int32 StunHeaderLength = 20;
        private UdpClient UdpClient;
        private IPEndPoint TurnEndPoint;
        private String Username;
        private String Password;
        private String Realm;
        private String Nonce;
        private Byte[] TransactionId;

        private static readonly String[] GoogleStunServers =
        {
            "stun.l.google.com:19302",
            "stun1.l.google.com:19302",
            "stun2.l.google.com:19302",
            "stun3.l.google.com:19302",
            "stun4.l.google.com:19302"
        };

        public TurnClient(String CustomServer)
        {
            UdpClient = new UdpClient();
            UdpClient.Client.ReceiveTimeout = 3000;

            String RandomServer = !String.IsNullOrEmpty(CustomServer) ? CustomServer : GoogleStunServers[new Random().Next(GoogleStunServers.Length)];
            String[] Parts = RandomServer.Split(':');
            String Server = Parts[0];
            Int32 Port = Int32.Parse(Parts[1]);

            TurnEndPoint = new IPEndPoint(Dns.GetHostAddresses(Server)[0], Port);
            TransactionId = new Byte[12];
            new Random().NextBytes(TransactionId);

            Username = "";
            Password = "";
        }

        public async Task<IPEndPoint> GetReflexiveAddressAsync()
        {
            try
            {
                Byte[] Request = BuildBindingRequest();
                await UdpClient.SendAsync(Request, Request.Length, TurnEndPoint);

                UdpReceiveResult Response = await UdpClient.ReceiveAsync();
                return ParseMappedAddress(Response.Buffer);
            }
            catch (Exception Exception)
            {
                Console.WriteLine($"STUN request failed: {Exception.Message}");
                return null;
            }
        }

        private Byte[] BuildBindingRequest()
        {
            Byte[] Message = new Byte[StunHeaderLength];

            Message[0] = 0x00;
            Message[1] = 0x01;

            Message[2] = 0x00;
            Message[3] = 0x00;

            Message[4] = 0x21;
            Message[5] = 0x12;
            Message[6] = 0xA4;
            Message[7] = 0x42;

            Buffer.BlockCopy(TransactionId, 0, Message, 8, 12);

            return Message;
        }

        private IPEndPoint ParseMappedAddress(Byte[] Response)
        {
            try
            {
                const Int32 HeaderSize = 20;
                Int32 Offset = HeaderSize;

                while (Offset + 4 <= Response.Length)
                {
                    UInt16 AttributeType = (UInt16)IPAddress.NetworkToHostOrder(BitConverter.ToInt16(Response, Offset));
                    UInt16 AttributeLength = (UInt16)IPAddress.NetworkToHostOrder(BitConverter.ToInt16(Response, Offset + 2));
                    Offset += 4;

                    if (AttributeType == 0x0001) // MAPPED-ADDRESS
                    {
                        Byte Family = Response[Offset + 1];
                        UInt16 Port = (UInt16)(Response[Offset + 2] << 8 | Response[Offset + 3]);
                        Byte[] IpBytes = new Byte[4];
                        Array.Copy(Response, Offset + 4, IpBytes, 0, 4);

                        return new IPEndPoint(new IPAddress(IpBytes), Port);
                    }
                    else if (AttributeType == 0x0020) // XOR-MAPPED-ADDRESS
                    {
                        Byte Family = Response[Offset + 1];
                        UInt16 XorPort = (UInt16)(Response[Offset + 2] << 8 | Response[Offset + 3]);
                        UInt16 Port = (UInt16)(XorPort ^ 0x2112);

                        Byte[] IpBytes = new Byte[4];
                        Array.Copy(Response, Offset + 4, IpBytes, 0, 4);
                        Byte[] MagicCookie = BitConverter.GetBytes(0x2112A442);

                        for (int i = 0; i < 4; i++)
                        {
                            IpBytes[i] ^= MagicCookie[i];
                        }

                        return new IPEndPoint(new IPAddress(IpBytes), Port);
                    }

                    Offset += AttributeLength;
                    if (AttributeLength % 4 != 0)
                    {
                        Offset += 4 - (AttributeLength % 4); // padding
                    }
                }
            }
            catch (Exception Ex)
            {
                Console.WriteLine("STUN parse error: " + Ex.Message);
            }

            return null;
        }
    }
}