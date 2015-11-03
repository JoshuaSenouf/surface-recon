using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Text;
using System.IO;
using System;


public class IBEOSensor : MonoBehaviour {
	public int angle = 20;
	public int sampleRate = 10;

	public float length = 50;

	public bool sphereTest = false;
	public bool logTest = false;
	public bool dataStoringTest = false;

	private List<string[]> dataCSV = new List<string[]>();	// List that will store all our points coordinates


	void Start () {
		transform.Rotate (0, ((int)angle / 2) + 1, 0);	// We rotate the prefab on which the script is linked to, allowing it to scan in front of it.
		InvokeRepeating ("layerScanning", (float)0.0, (float)1.0 / sampleRate);	// Repeat the named method "sampleRate" times per second

		if (dataStoringTest){
			string[] columnTitleCSV = new string[3];	// We give the names of our columns to our first row's values
			columnTitleCSV [0] = "X";
			columnTitleCSV [1] = "Y";
			columnTitleCSV [2] = "Z";
			dataCSV.Add (columnTitleCSV);
		}
	}


	void Update() {
	}


	void layerScanning() {
		for (int i = 0; i < angle; i++) {
			transform.Rotate (0, -1, 0);	// We rotate the prefab from one degree and we call the pointCompute method. This will be repeated "angle" times.
			pointCompute ();
		}
		transform.Rotate (0, angle, 0);	// The prefab go back to its inital position, making it ready for the next call.
	}


	void pointCompute() {
		RaycastHit hit = new RaycastHit ();

		if (Physics.Raycast (transform.position, transform.forward, out hit, length)) {	// If the ray DID hit something...
			float angleY = (transform.eulerAngles.y) * Mathf.Deg2Rad;	// Mandatory since Mathf trigonometric functions only accept radians
			float angleZ = (transform.eulerAngles.z) * Mathf.Deg2Rad;
			float pointX = transform.position.x + hit.distance * Mathf.Cos(angleZ) * Mathf.Sin(angleY);
			float pointY = transform.position.y + hit.distance * Mathf.Sin (angleZ);	// Buggy at the moment ? Work well with a purely horizontal layer but starts to behave strangely if it is not the case
			float pointZ = transform.position.z + hit.distance * Mathf.Cos(angleZ) * Mathf.Cos(angleY);

			if (dataStoringTest){
				string[] tempDataCSV = new string[3];	// We store the 3 coordinates that were computed above
				tempDataCSV[0] = pointX.ToString();
				tempDataCSV[1] = pointY.ToString();
				tempDataCSV[2] = pointZ.ToString();
				dataCSV.Add(tempDataCSV);	// We add these datas in our list
			}

			if (sphereTest){	// Will spawn a small colliderless sphere at the impact point of the ray
				GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
				Collider coll = sphere.GetComponent<Collider>();
				coll.enabled = false;
				sphere.transform.position = new Vector3 (pointX, pointY, pointZ);	// Use the coordinates that were compute above
				//sphere.transform.position = hit.point;	// Use the coordinates provided by Unity
			}

			if (logTest) {	// Make the log print every relevant data in the console
				Debug.Log("Position : " +transform.position+" // Dist hit : " +hit.distance+ " // Impact : " +hit.point+ " // X : " +pointX+" // Y : " +pointY+" // Z : " +pointZ );
				//Debug.Log("Impact : " +hit.point+ "X : " +pointX+" // Y : " +pointY+" // Z : " +pointZ );
			}
		}
	}


	void OnGUI() {
		GUI.Box (new Rect (0, 0, 80, 25), (1 / Time.deltaTime).ToString ());	// Native (and basic) FPS Counter
	}

	
	void OnApplicationQuit() {
		if (dataStoringTest) {
			string[][] outputData = new string[dataCSV.Count][];	// We create a 2-dimension output list, with its first dimension being as large as the number of rows our main list contains
			
			for (int i = 0; i < outputData.Length; i++) {
				outputData [i] = dataCSV [i];	// We write every elements (row) of the main list in our output list, each one to its corresponding index
			}
			
			int outputLength = outputData.GetLength (0);	// We get the length of the first dimension of our output list
			string delimitChar = ",";
			StringBuilder buffer = new StringBuilder ();	// We use a StringBuilder as a buffer and not a StringBuffer. The difference is that unlike StringBuffer, StringBuilder does not feature synchronization, which is useless here and quite costly performance-wise
			
			for (int i = 0; i < outputLength; i++)
				buffer.AppendLine (string.Join (delimitChar, outputData [i]));	// We add every sublist from our main list in the buffer, so that we can write them in a file later
			
			string filePath = Application.dataPath + "/CSV/" + "dataPoints.csv";
			
			StreamWriter outStream = System.IO.File.CreateText (filePath);	// We create the csv file, and overwrite it if it already exists. Caution with the R/W rights !
			outStream.WriteLine (buffer);	// We write the content of our buffer in the csv file
			outStream.Close ();
		}
	}
}
