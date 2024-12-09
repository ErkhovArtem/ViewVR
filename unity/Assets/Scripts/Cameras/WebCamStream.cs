using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using static System.BitConverter;
using static System.Buffer;
using System.IO;
using static System.String;
using static System.DateTime;

public class WebCamStream : MonoBehaviour
{
    public RawImage rawImage;
    public bool usePredefinedCameras;
    public string[] deviceNames;
    public int currentIndex = 0;

    private WebCamDevice[] devices;
    private WebCamTexture webcamTexture;

    public int connectionPort = 25001;
    TcpListener server;
    TcpClient client;
    bool running;
    Thread thread;
    private byte[] bytes = null;
    public int flag;
    public string path = " ";
    private int count;
    private string current_dir;

    void Start()
    {
        // Render to texture
        webcamTexture = new WebCamTexture();
        rawImage.texture = webcamTexture;
        rawImage.material.mainTexture = webcamTexture;

        // Get all connected devices
        devices = WebCamTexture.devices;
        if (devices.Length == 0)
        {
            Debug.Log("No camera is connected");
        }

        // If no predefined names are given
        if (!usePredefinedCameras)
        {
            deviceNames = new string[devices.Length];
            for (int i = 0; i < devices.Length; ++i)
            {
                deviceNames[i] = devices[i].name;
            }
        }

        // separate thread for data receiving from Python
        ThreadStart ts = new ThreadStart(GetData);
        thread = new Thread(ts);
        thread.Start();

        current_dir = CreateDir(path);
        LoadCamera(0);
    }

    void GetData()
    {
        // Create the server
        server = new TcpListener(IPAddress.Any, connectionPort);
        server.Start();

        // Create a client to get the data stream
        client = server.AcceptTcpClient();

        // Start listening
        running = true;
        while (running)
        {
            Connection();
        }
        server.Stop();
    }

    void Connection()
    {
        // Read data from the network stream
        NetworkStream nwStream = client.GetStream();  
        byte[] buffer = new byte[client.ReceiveBufferSize];
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize);
        flag = System.BitConverter.ToInt32(buffer, 0);

    }

    void Update()
    {
        if (devices.Length == 0)
            return;
        
        // Increment index
        if (Input.GetKeyDown(KeyCode.Space))
        {
            currentIndex = (currentIndex+1) % deviceNames.Length;
            LoadCamera(currentIndex);
        }

        if (flag == 1)
        {
            flag = 0;
            count+=1;
            Texture2D texture = new Texture2D(webcamTexture.width, webcamTexture.height);
            color32 = webcamTexture.GetPixels32();
            texture.SetPixels32(color32);
            texture.Apply();
            bytes = texture.EncodeToPNG();
            string current_path = string.Format(current_dir + "photo_{0}.png", count);
            File.WriteAllBytes(current_path, bytes);
        }

        if (flag ==2) {
            flag = 0;
            count = 0;
            current_dir = CreateDir(path);
        } 
        // string base64string = Convert.ToBase64String(bytes);
        // byte[] data = Encoding.UTF8.GetBytes(base64string);

        // NetworkStream nwStream = client.GetStream();
        // nwStream.Write(bytes, 0, bytes.Length);
        
        // frame transmitting
        // NetworkStream nwStream = client.GetStream();
        // float a = 1f;
        // float b = 2f;
        // var floatArray = new float[] {a, b};
        // var byteArray = new byte[floatArray.Length * 4];
        // System.Buffer.BlockCopy(floatArray, 0, byteArray, 0, byteArray.Length);
        // nwStream.Write(byteArray, 0, 8);
    }

    public void LoadCamera(int index)
    {
        if (index < deviceNames.Length)
        {
            currentIndex = index;
            // change texture
            webcamTexture.Stop();
            webcamTexture.deviceName = deviceNames[currentIndex];
            webcamTexture.Play();
        }
    }

    private string CreateDir(string Folder)
{
    // Sample on Sun 01.12.2014 At 09.13 AM
    string locationToCreateFolder = Folder;
    string current_datetime = Now.ToString("yyyyMMdd_HHmmss");
    // folderName = string.Format(format, date, time);
    Directory.CreateDirectory(locationToCreateFolder + current_datetime + "/");
    return locationToCreateFolder + current_datetime + "/";
}
}
