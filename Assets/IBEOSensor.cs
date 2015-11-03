using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System;
using System.IO;


public class IBEOSensor : MonoBehaviour {
	public int angle = 20;
	public int sampleRate = 10;
	public float length = 50;
	public bool sphereTest = false;
	public bool logTest = false;


	void Start () {
		transform.Rotate (0, ((int)angle / 2) + 1, 0);	// We rotate the prefab on which the script is linked to, allowing it to scan in front of it.
		InvokeRepeating("layerScanning", (float) 0.0, (float) 1.0/sampleRate);	// Repeat the named method "sampleRate" times per second
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

			if (sphereTest){	// Will spawn a small colliderless sphere at the impact point of the ray
				GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
				Collider coll = sphere.GetComponent<Collider>();
				coll.enabled = false;
				sphere.transform.position = new Vector3 (pointX, pointY, pointZ);	// Use the coordinates that were compute above
				//sphere.transform.position = hit.point;	// Use the coordinates provided by Unity
			}

			if (logTest) {	// Make the log print every relevant data in the console
				Debug.Log("Position : " +transform.position+" // Dist hit : " +hit.distance+ " // Impact : " +hit.point+ "X : " +pointX+" // Y : " +pointY+" // Z : " +pointZ );
				//Debug.Log("Impact : " +hit.point+ "X : " +pointX+" // Y : " +pointY+" // Z : " +pointZ );
			}
		}
	}


	void OnGUI() {
		GUI.Box (new Rect (0, 0, 80, 25), (1 / Time.deltaTime).ToString ());	// Native FPS Counter
	}
}
