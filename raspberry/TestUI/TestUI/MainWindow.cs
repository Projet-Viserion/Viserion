using System;
using Gtk;
using System.Net.Sockets;
using System.Text;


public partial class MainWindow : Gtk.Window
{
    TcpClient clientSocket = new TcpClient();
    NetworkStream nwStream;

    public MainWindow() : base(Gtk.WindowType.Toplevel)
    {
        Build();
    }

    protected void OnDeleteEvent(object sender, DeleteEventArgs a)
    {
        Application.Quit();
        a.RetVal = true;
    }

    protected void OnHscale4ValueChanged(object sender, EventArgs e)
    {
        byte[] bytes = Encoding.ASCII.GetBytes("SPE" + hscale4.Value.ToString());

        nwStream.Write(bytes, 0, bytes.Length);
    }

    protected void OnBtconnectClicked(object sender, EventArgs e)
    {
        try
        {
            // Connect the socket to the server
            clientSocket.Connect(ip.Text, 6666);

            btconnect.Label = "Connected";
            btconnect.State = StateType.Insensitive;
            nwStream = clientSocket.GetStream();
            // Send initial speed value
            byte[] bytes = Encoding.ASCII.GetBytes("SPE" + hscale4.Value.ToString());
            nwStream.Write(bytes, 0, bytes.Length);
            Receive();


        }
        catch (SocketException ex)
        {
            btconnect.Label = "Failed to connect";
            Console.WriteLine(ex.Message); 
        }
    }

    async void Receive(){
        int cmpt = 0;
        while (clientSocket.Connected)
        {
            byte[] myReadBuffer = new byte[2048];
            await nwStream.ReadAsync(myReadBuffer, 0, myReadBuffer.Length);
            String st = Encoding.UTF8.GetString(myReadBuffer);
            String[] msgs = st.Split(';');

            foreach (String msg in msgs){
                Console.WriteLine(msg);
                String[] elt = msg.Split(':');
                switch (elt[0])
                {
                    /*case "UFL":
                        USFL.LabelProp = elt[1];
                        break;
                    case "UFC":
                        USFC.LabelProp = elt[1];
                        break;
                    case "UFR":
                        USFR.LabelProp = elt[1];
                        break;
                    case "URL":
                        USRL.LabelProp = elt[1];
                        break;
                    case "URC":
                        USRC.LabelProp = elt[1];
                        break;
                    case "URR":
                        USRR.LabelProp = elt[1];
                        break;
                    case "POS":
                        ePOS.Text = elt[1];
                        break;
                    case "BAT":
                        eBAT.Text = elt[1];
                        break;
                    case "SWL":
                        eSPL.Text = elt[1];
                        break;
                    case "SWR":
                        eSPR.Text = elt[1];
                        break;*/
                    case "YAW":
                        eYAW.Text = elt[1];
                        break;
                    case "ROL":
                        eROL.Text = elt[1];
                        break;
                    case "PIT":
                        ePIT.Text = elt[1];
                        break;
                    default:
                        cmpt = (cmpt + 1) % 100;
                        break;
                }
                if (cmpt == 0)
                {
                    switch (elt[0])
                    {
                        case "UFL":
                            USFL.LabelProp = elt[1];
                            break;
                        case "UFC":
                            USFC.LabelProp = elt[1];
                            break;
                        case "UFR":
                            USFR.LabelProp = elt[1];
                            break;
                        case "URL":
                            USRL.LabelProp = elt[1];
                            break;
                        case "URC":
                            USRC.LabelProp = elt[1];
                            break;
                        case "URR":
                            USRR.LabelProp = elt[1];
                            break;
                        case "POS":
                            ePOS.Text = elt[1];
                            break;
                        case "BAT":
                            eBAT.Text = elt[1];
                            break;
                        case "SWL":
                            eSPL.Text = elt[1];
                            break;
                        case "SWR":
                            eSPR.Text = elt[1];
                            break;
                    }
                }
            }
        }

    }

    protected void OnButton1Clicked(object sender, EventArgs e)
    {
        byte[] bytes = Encoding.ASCII.GetBytes("MOV" + "forward");
        nwStream.Write(bytes, 0, bytes.Length);
    }

    protected void OnButton2Clicked(object sender, EventArgs e)
    {
        byte[] bytes = Encoding.ASCII.GetBytes("MOV" + "stop");
        nwStream.Write(bytes, 0, bytes.Length);
    }

    protected void OnButton4Clicked(object sender, EventArgs e)
    {
        byte[] bytes = Encoding.ASCII.GetBytes("MOV" + "backward");
        nwStream.Write(bytes, 0, bytes.Length);
    }

    protected void OnButton7Clicked(object sender, EventArgs e)
    {
        byte[] bytes = Encoding.ASCII.GetBytes("STE" + "stop");
        nwStream.Write(bytes, 0, bytes.Length);
    }

    protected void OnButton3Clicked(object sender, EventArgs e)
    {
        byte[] bytes = Encoding.ASCII.GetBytes("STE" + "right");
        nwStream.Write(bytes, 0, bytes.Length);
    }

    protected void OnButton5Clicked(object sender, EventArgs e)
    {
        byte[] bytes = Encoding.ASCII.GetBytes("STE" + "left");
        nwStream.Write(bytes, 0, bytes.Length);
    }


}
