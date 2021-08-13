<?php 
 $link = mysqli_connect("localhost", "arjuna", "arjuna", "db_robot");

 if (!$link) {
 	echo "Error: Unable to connect to MySQL.";
 	exit;
 }
 else {
	echo "Berhasil";
 }
 
 $robot_id = $_GET['id'];
 $status = $_GET['status'];
 
 mysqli_query($link,"UPDATE tbl_robot set status = '$status' where id = '$robot_id'");
?>


