using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Open.Nat;
namespace SoftwareRenderer
{
    public class Networking
    {
        private UdpClient UdpClient;
        private IPEndPoint RemoteEndPoint;
        private bool IsHost = false;
        private int ListeningPort;
        private CancellationTokenSource ReceiveCancellationTokenSource;
        public int ClientId = 0;
        private int NextClientId = 1;
        private ConcurrentDictionary<IPEndPoint, int> ConnectedClients = new();
        private ConcurrentQueue<string> BufferedRpcs = new();

        public bool IsConnected => UdpClient != null;
        public bool IsHosting => IsHost;

        public event Action<string, string[]> OnReceiveRPC;

        private TaskCompletionSource<bool> ClientIdAssignedTcs;
        
        private async Task<bool> TryEnableUpnpAsync(int port)
        {
            try
            {
                var nat = new NatDiscoverer();
                var cts = new CancellationTokenSource(5000);
                var device = await nat.DiscoverDeviceAsync(PortMapper.Upnp, cts);

                var ip = await device.GetExternalIPAsync();
                Console.WriteLine($"[UPnP] External IP: {ip}");

                await device.CreatePortMapAsync(new Mapping(Protocol.Udp, port, port, "SoftwareRendererHost"));
                Console.WriteLine($"[UPnP] Port {port} forwarded via UPnP.");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[UPnP] Failed to enable UPnP: {ex.Message}");
                return false;
            }
        }
        
        
        private async Task TryRemoveUpnpMappingAsync(int port)
        {
            try
            {
                var nat = new NatDiscoverer();
                var cts = new CancellationTokenSource(3000);
                var device = await nat.DiscoverDeviceAsync(PortMapper.Upnp, cts);
                await device.DeletePortMapAsync(new Mapping(Protocol.Udp, port, port));
                Console.WriteLine($"[UPnP] Port {port} mapping removed.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[UPnP] Failed to remove port mapping: {ex.Message}");
            }
        }

public async Task<bool> Connect(string host = "127.0.0.1", int port = 7777)
{
    ListeningPort = port;

    // Resolve domain to IP address
    IPAddress[] hostAddresses;
    try
    {
        hostAddresses = await Dns.GetHostAddressesAsync(host);
    }
    catch (Exception ex)
    {
        Console.WriteLine($"Failed to resolve host '{host}': {ex.Message}");
        return false;
    }

    IPAddress ipAddress = hostAddresses.FirstOrDefault();
    Console.WriteLine("Address Found: " + ipAddress);
    if (ipAddress == null)
    {
        Console.WriteLine($"No IP address found for host '{host}'.");
        return false;
    }

    RemoteEndPoint = new IPEndPoint(ipAddress, port);
    ReceiveCancellationTokenSource = new CancellationTokenSource();

    bool receivedHandshake = false;

    // ReuseAddress enabled for the temporary client
    using (var tempClient = new UdpClient())
    {
        tempClient.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
        tempClient.Client.Bind(new IPEndPoint(IPAddress.Any, 0)); // Bind to any free port

        tempClient.Client.ReceiveTimeout = 1500;

        try
        {
            byte[] handshake = Encoding.UTF8.GetBytes("ping");
            await tempClient.SendAsync(handshake, handshake.Length, RemoteEndPoint);

            var receiveTask = tempClient.ReceiveAsync();
            var delayTask = Task.Delay(1000);
            var completedTask = await Task.WhenAny(receiveTask, delayTask);

            if (completedTask == receiveTask)
            {
                var response = await receiveTask;
                string message = Encoding.UTF8.GetString(response.Buffer);
                if (message == "pong")
                {
                    receivedHandshake = true;
                }
            }
        }
        catch (SocketException ex) when (ex.SocketErrorCode == SocketError.TimedOut)
        {
            Console.WriteLine("Connection attempt timed out. No pong received.");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Error during connection attempt: {ex.Message}");
        }
    }

    if (receivedHandshake)
    {
        Console.WriteLine("Connected as client.");

        // ReuseAddress enabled for main UdpClient
        UdpClient = new UdpClient();
        UdpClient.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
        UdpClient.Connect(RemoteEndPoint);

        StartReceiveLoop();

        ClientIdAssignedTcs = new TaskCompletionSource<bool>();
        SendRPC("Ping", new string[] { "0" });
        await ClientIdAssignedTcs.Task;
        return true;
    }
    else
    {
        Console.WriteLine("No response - becoming host...");

        bool upnpSuccess = await TryEnableUpnpAsync(ListeningPort);

        if (!upnpSuccess)
        {
            Console.WriteLine("Warning: Failed to map port via UPnP.");
        }

        await Task.Delay(1000);

        try
        {
            UdpClient = new UdpClient();
            UdpClient.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
            UdpClient.Client.Bind(new IPEndPoint(IPAddress.Any, ListeningPort));

            IsHost = true;
            Console.WriteLine($"Listening for connections on port {ListeningPort}");
            StartReceiveLoop();
            return true;
        }
        catch (SocketException ex)
        {
            Console.WriteLine($"Error binding to port {ListeningPort}: {ex.Message}. Is another instance already running?");
            Close();
            return false;
        }
    }
}

        internal void Send(string Message, IPEndPoint TargetEndPoint = null)
        {
            if (UdpClient == null)
            {
                Console.WriteLine("Cannot send: UDP client not initialized.");
                return;
            }

            byte[] Data = Encoding.UTF8.GetBytes(Message);

            if (IsHost)
            {
                if (TargetEndPoint != null)
                {
                    try
                    {
                        UdpClient.Send(Data, Data.Length, TargetEndPoint);
                    }
                    catch (Exception Ex)
                    {
                        Console.WriteLine($"Error sending to client {TargetEndPoint}: {Ex.Message}");
                    }
                }
                else
                {
                    if (ConnectedClients.IsEmpty)
                    {
                        Console.WriteLine("Cannot send: No connected clients.");
                        return;
                    }
                    foreach (var Client in ConnectedClients.Keys)
                    {
                        try
                        {
                            UdpClient.Send(Data, Data.Length, Client);
                        }
                        catch (Exception Ex)
                        {
                            Console.WriteLine($"Error sending to client {Client}: {Ex.Message}");
                        }
                    }
                }
            }
            else
            {
                try
                {
                    UdpClient.Send(Data, Data.Length);
                }
                catch (Exception Ex)
                {
                    Console.WriteLine($"Error sending data: {Ex.Message}");
                }
            }
        }

        public void SendRPC(string MethodName, string[] Parameters, int TargetClientId = 0, bool BufferRPC = false)
        {
            if (UdpClient == null)
            {
                Console.WriteLine("Cannot send RPC: UDP client not initialized.");
                return;
            }

            string RpcMessage = "RPC:" + MethodName;
            if (Parameters != null && Parameters.Length > 0)
            {
                RpcMessage += ":" + string.Join(":", Parameters);
            }

            if (!IsHost && ClientId != 0)
            {
                RpcMessage += $":senderId={ClientId}";
            }

            byte[] Data = Encoding.UTF8.GetBytes(RpcMessage);

            if (IsHost)
            {
                if (BufferRPC)
                {
                    BufferedRpcs.Enqueue(RpcMessage);
                    Console.WriteLine($"Buffered RPC: {RpcMessage}");
                }

                if (TargetClientId == 0)
                {
                    foreach (var Client in ConnectedClients.Keys)
                    {
                        try
                        {
                            UdpClient.Send(Data, Data.Length, Client);
                        }
                        catch (Exception Ex)
                        {
                            Console.WriteLine($"Error broadcasting RPC to client {Client}: {Ex.Message}");
                        }
                    }

                    OnReceiveRPC?.Invoke(MethodName, Parameters);
                }
                else
                {
                    var TargetClient = ConnectedClients.FirstOrDefault(Kv => Kv.Value == TargetClientId);
                    if (!TargetClient.Equals(default(KeyValuePair<IPEndPoint, int>)))
                    {
                        try
                        {
                            UdpClient.Send(Data, Data.Length, TargetClient.Key);
                        }
                        catch (Exception Ex)
                        {
                            Console.WriteLine($"Error sending RPC to client {TargetClientId} ({TargetClient.Key}): {Ex.Message}");
                        }
                    }
                    else
                    {
                        Console.WriteLine($"Cannot send RPC: Client with ID {TargetClientId} not found.");
                    }
                }
            }
            else
            {
                try
                {
                    UdpClient.Send(Data, Data.Length);
                    OnReceiveRPC?.Invoke(MethodName, Parameters);
                }
                catch (Exception Ex)
                {
                    Console.WriteLine($"Error sending RPC to host: {Ex.Message}");
                }
            }
        }

        private void StartReceiveLoop()
        {
            Task.Run(async () =>
            {
                try
                {
                    while (!ReceiveCancellationTokenSource.Token.IsCancellationRequested)
                    {
                        var Result = await UdpClient.ReceiveAsync();
                        string Message = Encoding.UTF8.GetString(Result.Buffer).Trim();
                        var Sender = Result.RemoteEndPoint;

                        Console.WriteLine($"Received: {Message} from {Sender}");

                        if (IsHost && Message == "ping")
                        {
                            await UdpClient.SendAsync(Encoding.UTF8.GetBytes("pong"), 4, Sender);
                            Console.WriteLine($"Replied 'pong' to {Sender}");
                            continue;
                        }
                        else if (!IsHost && Message == "pong")
                        {
                            continue;
                        }
                        else if (!IsHost && Message.StartsWith("id:"))
                        {
                            HandleClientIdAssignment(Message);
                            continue;
                        }

                        if (IsHost)
                        {
                            await HandleHostMessageAsync(Message, Sender);
                        }

                        if (Message.StartsWith("RPC:"))
                        {
                            ParseAndInvokeRPC(Message, Sender);
                        }
                    }
                }
                catch (OperationCanceledException)
                {
                    Console.WriteLine("Receive loop cancelled.");
                }
                catch (ObjectDisposedException)
                {
                    Console.WriteLine("UdpClient was disposed, stopping receive loop.");
                }
                catch (Exception Ex)
                {
                    Console.WriteLine($"Receive error: {Ex.Message}");
                }
            }, ReceiveCancellationTokenSource.Token);
        }

        private void ParseAndInvokeRPC(string RpcMessage, IPEndPoint Sender)
        {
            var Parts = RpcMessage.Split(':');
            if (Parts.Length < 2 || Parts[0] != "RPC")
            {
                Console.WriteLine($"Invalid RPC format: {RpcMessage}");
                return;
            }

            string MethodName = Parts[1];
            string[] Parameters = Parts.Skip(2).ToArray();

            int RpcSenderClientId = 0;
            if (!IsHost && Parameters.Length > 0 && Parameters.Last().StartsWith("senderId="))
            {
                var SenderIdString = Parameters.Last().Substring("senderId=".Length);
                if (int.TryParse(SenderIdString, out int ParsedSenderId))
                {
                    RpcSenderClientId = ParsedSenderId;
                    Parameters = Parameters.Take(Parameters.Length - 1).ToArray();
                }
            }
            else if (IsHost && ConnectedClients.ContainsKey(Sender))
            {
                RpcSenderClientId = ConnectedClients[Sender];
            }

            if (IsHost)
            {
                switch (MethodName)
                {
                    case "Ping":
                        HandleHostPingRPC(Parameters, Sender, RpcSenderClientId);
                        return;
                    case "Disconnect":
                        HandleHostDisconnectRPC(Parameters, Sender);
                        return;
                }
            }
            else
            {
                switch (MethodName)
                {
                    case "Disconnect":
                        HandleClientDisconnectRPC(Parameters);
                        return;
                }
            }

            OnReceiveRPC?.Invoke(MethodName, Parameters);
        }

        private async void HandleHostPingRPC(string[] Parameters, IPEndPoint Sender, int RpcSenderClientId)
        {
            if (Parameters.Length >= 1 && int.TryParse(Parameters[0], out int ClientIdFromRpc))
            {
                if (ClientIdFromRpc == 0)
                {
                    int NewClientId = NextClientId++;
                    ConnectedClients[Sender] = NewClientId;
                    Console.WriteLine($"New client connected: {Sender} assigned ID {NewClientId}");
                    Send("id:" + NewClientId, Sender);
                    foreach (var BufferedRpc in BufferedRpcs)
                    {
                        try
                        {
                            byte[] Data = Encoding.UTF8.GetBytes(BufferedRpc);
                            await UdpClient.SendAsync(Data, Data.Length, Sender);
                            Console.WriteLine($"Sent buffered RPC to new client: {BufferedRpc}");
                        }
                        catch (Exception Ex)
                        {
                            Console.WriteLine($"Error sending buffered RPC to new client: {Ex.Message}");
                        }
                    }
                }
                else
                {
                    var ExistingEntry = ConnectedClients.FirstOrDefault(Kv => Kv.Value == ClientIdFromRpc);
                    if (!ExistingEntry.Equals(default(KeyValuePair<IPEndPoint, int>)) && !ExistingEntry.Key.Equals(Sender))
                    {
                        ConnectedClients.TryRemove(ExistingEntry.Key, out _);
                        ConnectedClients[Sender] = ClientIdFromRpc;
                        Console.WriteLine($"Updated client {ClientIdFromRpc} endpoint to {Sender}");
                    }
                    else if (ExistingEntry.Equals(default(KeyValuePair<IPEndPoint, int>)))
                    {
                        int NewClientId = NextClientId++;
                        ConnectedClients[Sender] = NewClientId;
                        Console.WriteLine($"Client {Sender} pinged with unknown ID {ClientIdFromRpc}. Re-assigned ID {NewClientId}");
                        Send("id:" + NewClientId, Sender);
                    }
                }
            }
            else
            {
                Console.WriteLine($"Host received malformed Ping RPC from {Sender}: {string.Join(":", Parameters)}");
            }
        }

        private void HandleHostDisconnectRPC(string[] Parameters, IPEndPoint Sender)
        {
            if (Parameters.Length >= 1 && int.TryParse(Parameters[0], out int ClientId))
            {
                var ClientEntry = ConnectedClients.FirstOrDefault(Kv => Kv.Value == ClientId);
                if (!ClientEntry.Equals(default(KeyValuePair<IPEndPoint, int>)))
                {
                    ConnectedClients.TryRemove(ClientEntry.Key, out _);
                    Console.WriteLine($"Client {ClientId} disconnected from {ClientEntry.Key}");
                    SendRPC("ClientDisconnected", new string[] { ClientId.ToString() });
                }
                else
                {
                    Console.WriteLine($"Host received Disconnect RPC for unknown client ID {ClientId} from {Sender}");
                }
            }
            else
            {
                Console.WriteLine($"Host received malformed Disconnect RPC from {Sender}: {string.Join(":", Parameters)}");
            }
        }

        private void HandleClientIdAssignment(string Message)
        {
            var Parts = Message.Split(':');
            if (Parts.Length == 2 && int.TryParse(Parts[1], out int AssignedId))
            {
                ClientId = AssignedId;
                Console.WriteLine($"Assigned client ID: {ClientId}");

                // Signal that the ClientId has been assigned
                ClientIdAssignedTcs?.TrySetResult(true);
            }
        }

        private void HandleClientDisconnectRPC(string[] Parameters)
        {
            if (Parameters.Length >= 1 && int.TryParse(Parameters[0], out int DisconnectedClientId))
            {
                Console.WriteLine($"Client received notification: Client {DisconnectedClientId} disconnected.");
            }
            else
            {
                Console.WriteLine($"Client received malformed Disconnect RPC: {string.Join(":", Parameters)}");
            }
        }

        private async Task HandleHostMessageAsync(string Message, IPEndPoint Sender)
        {
            if (!Message.StartsWith("ping") && !Message.StartsWith("RPC:"))
            {
                Console.WriteLine($"Unhandled host message: {Message} from {Sender}");
            }
        }

        private void HandleClientMessage(string Message)
        {
            if (!Message.StartsWith("pong") && !Message.StartsWith("id:") && !Message.StartsWith("RPC:"))
            {
                Console.WriteLine($"Unhandled client message: {Message}");
            }
        }

        public void ClearBufferedRpcs()
        {
            BufferedRpcs = new ConcurrentQueue<string>();
            Console.WriteLine("Cleared all buffered RPCs");
        }

        public void Close()
        {
            if (IsHost)
            {
                _ = TryRemoveUpnpMappingAsync(ListeningPort);
            }
            Console.WriteLine($"Closing networking. My ClientId: {ClientId}");
            try
            {
                if (UdpClient != null)
                {
                    if (!IsHost && ClientId != 0)
                    {
                        SendRPC("Disconnect", new string[] { ClientId.ToString() });
                    }
                }
            }
            catch (Exception Ex)
            {
                Console.WriteLine($"Error sending disconnect RPC: {Ex.Message}");
            }

            ReceiveCancellationTokenSource?.Cancel();

            UdpClient?.Close();
            UdpClient = null;

            Console.WriteLine("Networking client closed.");
        }
    }
}