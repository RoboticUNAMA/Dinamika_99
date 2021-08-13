<?php 
 $link = mysqli_connect("localhost", "arjuna", "arjuna", "db_robot");

 if (!$link) {
 	echo "Error: Unable to connect to MySQL.";
 	exit;
 }
 
 $robot_id = $_GET['id'];
 $query = mysqli_query($link,"SELECT status FROM tbl_robot WHERE id = '$robot_id'");
 $row = mysqli_fetch_array($query,MYSQLI_ASSOC);
 
 echo $row['status'];
?>


