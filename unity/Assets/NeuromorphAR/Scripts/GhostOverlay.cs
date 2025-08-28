using UnityEngine;

public class GhostOverlay : MonoBehaviour
{
    [Range(0f,2f)] public float popExponent = 1.6f;   // non-linear pop
    [Range(0f,1f)] public float flickerBelow = 0.25f; // flicker threshold
    public float flickerHz = 6f;

    private Material mat;

    void Awake(){ mat = GetComponent<Renderer>()?.material; }

    public void UpdateGhost(float confidence, Color baseColor)
    {
        float curved = Mathf.Pow(Mathf.Clamp01(confidence), popExponent);
        float alpha = Mathf.Lerp(0.1f, 1.0f, curved);

        // flicker when uncertain
        if (confidence < flickerBelow)
        {
            float phase = Mathf.Abs(Mathf.Sin(Time.time * Mathf.PI * flickerHz));
            alpha *= Mathf.Lerp(0.4f, 0.9f, phase);  // subtle shimmer
        }

        var c = baseColor; c.a = alpha;
        if (mat != null) { mat.SetColor("_Color", c); }
    }
}
