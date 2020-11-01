using UnityEngine;
using System.IO.Ports;
using System;
using System.Globalization;

public class GetRotation : MonoBehaviour
{
    // Start is called before the first frame update
    SerialPort sp;
    string rec;
    void Start()
    {
        sp = new SerialPort("COM6", 115200);
        
        sp.Open();        
    }

    // Update is called once per frame
    void Update()
    {
        try
        {
            rec = sp.ReadLine();
            Debug.Log(rec);
            if (rec.IndexOf(">>") != 0 && rec.IndexOf("<<") < rec.IndexOf(">>"))
            {
                string part = rec.Substring(rec.IndexOf("<<") + 2, rec.IndexOf(">>") - rec.IndexOf("<<") - 2);
                Debug.Log(part);
                string[] numS=part.Split(',');
                Quaternion q = new Quaternion();

                q.w = float.Parse(numS[0], CultureInfo.InvariantCulture);
                q.z = -float.Parse(numS[1], CultureInfo.InvariantCulture);
                q.x = float.Parse(numS[2], CultureInfo.InvariantCulture);
                q.y = -float.Parse(numS[3], CultureInfo.InvariantCulture);

                transform.rotation = q;
            }
        }
        catch (TimeoutException e)
        {
            Debug.Log(e.Message);
        }
    }
    private void OnDestroy()
    {
        sp.Close();
        sp.Dispose();
    }
}
