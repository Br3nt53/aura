using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System;

public class ParamSender : MonoBehaviour
{
    ROSConnection ros;
    [Tooltip("Topic watched by param_bridge.py")]
    public string topic = "/aura/param_set";

    void Start() { ros = ROSConnection.GetOrCreateInstance(); }

    public void SetDouble(string paramName, float value, string node="/aura_fusion")
    {
        var payload = $"{{\"node\":\"{node}\",\"name\":\"{paramName}\",\"value\":{value.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture)}}}";
        var msg = new StringMsg(payload);
        ros.Publish(topic, msg);
    }
}
