using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.UIElements;

public class bug_algorithm_2 : MonoBehaviour
{
    public float speed;
    private Rigidbody objRigidbody;
    public Vector3 direction;

    //public GameObject Obstacle_first_touch;

    public GameObject line;
    public Transform goal;
    // 일단 collider를 포함하는 긴 직선을 만들어서 obstacle의 경유지를 모두 만듬.
    // agent는 그 경유지들을 모두 경유하면 됨

    private float cooltime = 0f;

    public enum Mode
    {
        motion_to_goal, boundary_following, goto_stopover
    }

    public Mode mode;
    // Start is called before the first frame update
    void Start()
    {
        mode = Mode.motion_to_goal;
        CreateLine(gameObject.transform, goal);
    }

    // Update is called once per frame
    void Update()
    {
        cooltime -= Time.deltaTime;
        if (mode == Mode.motion_to_goal){
            direction = (goal.position - gameObject.transform.position).normalized;
            transform.Translate(direction * speed * Time.deltaTime);
        }
        else if (mode == Mode.boundary_following || mode == Mode.goto_stopover)
        {
            Vector3 targetDirection = Quaternion.Euler(0, 1f, 0) * direction;
            direction = Vector3.Lerp(direction, targetDirection, 1.0f);
            transform.Translate(direction * speed * Time.deltaTime);

        }
    }



    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.tag == "wall")
        {
            if (mode == Mode.motion_to_goal && cooltime < 0f)
            {
                mode = Mode.boundary_following;
            }

            direction = collision.contacts[0].normal;
            direction.y = 0;
            direction = direction.normalized;
            Quaternion rotation = Quaternion.Euler(0, 85, 0);
            direction = Vector3.Lerp(direction, rotation * direction, 0.5f).normalized;
        }

    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "line" && mode == Mode.boundary_following)
        {
            print("r");
            cooltime = 0.2f;
            mode = Mode.motion_to_goal;
        }

        if (other.gameObject.tag == "goal")
        {
            print("Success");
        }
    }


    private void CreateLine(Transform point_1, Transform point_2)
    {

        float distance = Vector3.Distance(point_1.position, point_2.position);

  
        Vector3 newScale = line.transform.localScale;
        newScale.z = distance;

        line.transform.localScale = newScale;
        line.transform.position = (point_1.position + point_2.position) / 2;

        line.transform.LookAt(point_2.position);
    }

}
