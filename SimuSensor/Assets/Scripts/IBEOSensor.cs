/* IBEOSensor for SimuSensor

Date: 09/01/2016 - Joshua Senouf - Initial Release
Version: 1.0
Comment: Support of multiple layers for the sensor, debugging options, can customize the sensor's specs, export cloud point to PCD

Revisions:

09/01/2016 - Joshua Senouf - Initial Release


**DISCLAIMER**
THIS MATERIAL IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING, BUT Not LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE, OR NON-INFRINGEMENT. SOME JURISDICTIONS DO NOT ALLOW THE
EXCLUSION OF IMPLIED WARRANTIES, SO THE ABOVE EXCLUSION MAY NOT
APPLY TO YOU. IN NO EVENT WILL I BE LIABLE TO ANY PARTY FOR ANY
DIRECT, INDIRECT, SPECIAL OR OTHER CONSEQUENTIAL DAMAGES FOR ANY
USE OF THIS MATERIAL INCLUDING, WITHOUT LIMITATION, ANY LOST
PROFITS, BUSINESS INTERRUPTION, LOSS OF PROGRAMS OR OTHER DATA ON
YOUR INFORMATION HANDLING SYSTEM OR OTHERWISE, EVEN If WE ARE
EXPRESSLY ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
*/

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Text;
using System.IO;
using System;


public class IBEOSensor : MonoBehaviour {
	private int angle = 85;
	private int sampleRate = 10;
	private float length = 200;
	private string fileName;
	private bool sphereTest = false;
	private bool logTest = false;
	private bool dataStoringTest = false;

	private List<string[]> dataPCD = new List<string[]>();	// List that will store all our points coordinates


	void Start () {
		transform.Rotate (0, ((int)angle / 2) + 1, 0);	// We rotate the prefab on which the script is linked to, allowing it to scan in front of it.
		InvokeRepeating ("layerScanning", (float)0.0, (float)1.0 / sampleRate);	// Repeat the named method "sampleRate" times per second
	}


	void Update() {
	}


	void layerScanning() {
		for (int row = 0; row < 4; row++) {
			for (int col = 0; col < angle; col++) {
				transform.Rotate (0, -1, 0);	// We rotate the prefab from one degree and we call the pointCompute method. This will be repeated "angle" times.
				pointCompute ();
			}
			transform.Rotate (2, angle, 0);		// We lower the prefab of 2 degrees then we repeat the previous operation (4 times in total)
		}
		transform.Rotate (-8, 0, 0);
	}
		

	void pointCompute() {
		RaycastHit hit = new RaycastHit ();

		if (Physics.Raycast (transform.position, transform.forward, out hit, length)) {	// If the ray DID hit something...
//			float angleY = (transform.eulerAngles.y) * Mathf.Deg2Rad;	// Mandatory since Mathf trigonometric functions only accept radians
//			float angleZ = (transform.eulerAngles.z) * Mathf.Deg2Rad;
//			float pointX = transform.position.x + hit.distance * Mathf.Cos(angleZ) * Mathf.Sin(angleY);
//			float pointY = transform.position.y + hit.distance * Mathf.Sin (angleZ);						// Buggy at the moment, work well with a purely horizontal layer but starts to behave strangely if it is not the case
//			float pointZ = transform.position.z + hit.distance * Mathf.Cos(angleZ) * Mathf.Cos(angleY);

			float pointX = hit.point.x;
			float pointY = hit.point.y;
			float pointZ = hit.point.z;

			if (dataStoringTest){
				string[] tempDataPCD = new string[3];	// We store the 3 coordinates that were computed above
				tempDataPCD[0] = pointX.ToString();
				tempDataPCD[1] = pointY.ToString();
				tempDataPCD[2] = pointZ.ToString();
				dataPCD.Add(tempDataPCD);	// We add these datas in our list
			}

			if (sphereTest){	// Will spawn a small colliderless sphere at the impact point of the ray
				GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
				Collider coll = sphere.GetComponent<Collider>();
				coll.enabled = false;
				sphere.transform.position = new Vector3 (pointX, pointY, pointZ);	// Use the coordinates that were computed above
			}

			if (logTest) {	// Make the log print every relevant data in the console
				Debug.Log("Position : " +transform.position+" // Dist hit : " +hit.distance+ " // Impact : " +hit.point+ " // X : " +pointX+" // Y : " +pointY+" // Z : " +pointZ );
			}
		}
	}


	void OnGUI() {
		GUI.Box (new Rect (0, 0, 80, 25), (Mathf.RoundToInt(1 / Time.deltaTime)).ToString ());	// Native (and basic) FPS Counter
	}

	
	void OnApplicationQuit() {
		if (dataStoringTest) {
			string[][] outputData = new string[dataPCD.Count][];	// We create a 2-dimension output list, with its first dimension being as large as the number of rows our main list contains
			
			for (int i = 0; i < outputData.Length; i++) {
				outputData [i] = dataPCD [i];	// We write every elements (row) of the main list in our output list, each one to its corresponding index
			}
			
			int outputLength = outputData.GetLength (0);	// We get the length of the first dimension of our output list
			string delimitChar = ",";
			StringBuilder buffer = new StringBuilder ();	// We use a StringBuilder as a buffer and not a StringBuffer. The difference is that unlike StringBuffer, StringBuilder does not feature synchronization, which is useless here and quite costly performance-wise
			buffer.AppendLine ("# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "+ outputLength +"\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "+ outputLength +"\nDATA ascii");	// We generate the basic structure of a PCD file based on the data stored in dataPCD

			for (int i = 0; i < outputLength; i++)
				buffer.AppendLine (string.Join (delimitChar, outputData [i]));	// We add every sublist from our main list in the buffer, so that we can write them in a file later
			
			string filePath = Application.dataPath + "/PCD/" + fileName +".pcd";	// The directory needs to be created beforehand
			
			StreamWriter outStream = System.IO.File.CreateText (filePath);	// We create the csv file, and overwrite it if it already exists. Caution with the R/W rights !
			outStream.WriteLine (buffer);	// We write the content of our buffer in the csv file
			outStream.Close ();
		}
	}
}
