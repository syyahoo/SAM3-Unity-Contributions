using System.Collections.Generic;
using UnityEngine;

namespace StrikerPC.Map
{
    [CreateAssetMenu(fileName = "VenueScanDefinition", menuName = "Drone Ops/Venue Scan Definition")]
    public sealed class VenueScanDefinition : ScriptableObject
    {
        [SerializeField] private string scanId;
        [SerializeField] private int version = 1;
        [SerializeField] private float minX;
        [SerializeField] private float maxX;
        [SerializeField] private float minZ;
        [SerializeField] private float maxZ;
        [SerializeField] private float floorCellSize = 0.1f;
        [SerializeField] private int floorWidth;
        [SerializeField] private int floorHeight;
        [SerializeField] private string floorDataPath = "Maps/venue-floor.bin";
        [SerializeField] private float[] boardToVenue = new float[16];

        public string ScanId => scanId;
        public int Version => version;
        public float MinX => minX;
        public float MaxX => maxX;
        public float MinZ => minZ;
        public float MaxZ => maxZ;
        public float FloorCellSize => floorCellSize;
        public int FloorWidth => floorWidth;
        public int FloorHeight => floorHeight;
        public string FloorDataPath => floorDataPath;
        public IReadOnlyList<float> BoardToVenue => boardToVenue;
    }
}
