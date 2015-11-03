using UnityEngine;
using System.Collections;


public class PlayerController : MonoBehaviour {

	public float speed = 20.0F;
	public float gravity = 20.0F;
	public float rotateSpeed = 0.25F;

	private Vector3 moveDirection = Vector3.zero;
	void Update() {
		CharacterController controller = GetComponent<CharacterController>();
		if (controller.isGrounded) {
			moveDirection = new Vector3(0, 0, Input.GetAxis("Vertical"));	// We retrieve the information on the Up and Down key and translate the data in a movement.
			moveDirection = transform.TransformDirection(moveDirection);
			moveDirection *= speed;

			if ( Input.GetKey(KeyCode.LeftArrow) ) {
				transform.Rotate(0, -rotateSpeed, 0);
			}
			
			if ( Input.GetKey(KeyCode.RightArrow) ) {
				transform.Rotate(0, rotateSpeed, 0);
			}
			
		}
		moveDirection.y -= gravity * Time.deltaTime;
		controller.Move(moveDirection * Time.deltaTime);	// The gravity and the rotate move will conflict with each other most of the time, bad method
	}
}
