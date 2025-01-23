using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.UIElements;

public class bug_algorithm_1 : MonoBehaviour
{
    public float speed;
    private Rigidbody objRigidbody;
    public Vector3 direction;

    public GameObject Obstacle_first_touch;
    public GameObject nearest_point;
    public Transform goal;


    private float cooltime = 0f;
    private bool hasTouched = false;



    public enum Mode
    {
        motion_to_goal, boundary_following, goto_nearest
    }

    public Mode mode;
    // Start is called before the first frame update
    void Start()
    {
        //objRigidbody = GetComponent<Rigidbody>();
        //objRigidbody.velocity = transform.forward * speed;
        mode = Mode.motion_to_goal;
        nearest_point.transform.position = new Vector3(100,100,100);
    }

    // Update is called once per frame
    void Update()
    {
        cooltime -= Time.deltaTime;

        if (mode == Mode.motion_to_goal){
            direction = (goal.position - gameObject.transform.position).normalized;
            transform.Translate(direction * speed * Time.deltaTime);
        }
        else if (mode == Mode.boundary_following || mode == Mode.goto_nearest)
        {

            //transform.Translate(direction * speed * Time.deltaTime);
            //Quaternion rotation = Quaternion.Euler(0, 0.01f, 0);
            //direction = rotation * direction;
            //direction = direction.normalized;
            Vector3 targetDirection = Quaternion.Euler(0, 1, 0) * direction;
            direction = Vector3.Lerp(direction, targetDirection, 2.0f);

            direction.y = 0;
            transform.Translate(direction * speed * Time.deltaTime);

            //nearest 지점 추적
            if (Vector3.Distance(nearest_point.transform.position, goal.position) > Vector3.Distance(gameObject.transform.position, goal.position))
            {
                nearest_point.transform.position = gameObject.transform.position;
            }

        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.tag == "wall")
        {
            if (mode == Mode.motion_to_goal && cooltime < 0f)
            {


                
                Obstacle_first_touch.transform.position = gameObject.transform.position;
                mode = Mode.boundary_following;
            }

            direction = collision.contacts[0].normal;
            direction.y = 0;
            direction = direction.normalized;
            Quaternion rotation = Quaternion.Euler(0, 85, 0);
            //direction = rotation * direction;
            //direction = direction.normalized;
            direction = Vector3.Lerp(direction, rotation * direction, 0.5f).normalized;
        }

    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "touch")
        {
            print("touch");
            if (hasTouched == true)
            {
                mode = Mode.goto_nearest;
            }
            hasTouched = true;
        }
        if (other.gameObject.tag == "nearest" && mode == Mode.goto_nearest)
        {
            cooltime = 0.2f;
            mode = Mode.motion_to_goal;
            hasTouched = false;
        }

        if (other.gameObject.tag == "goal")
        {
            print("Success");
        }
    }




}
