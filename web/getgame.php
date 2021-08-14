<?php 
 $link = mysqli_connect("localhost", "arjuna", "arjuna", "db_robot");

 if (!$link) {
 	echo "Error: Unable to connect to MySQL.";
 	exit;
 }

 $query = mysqli_query($link,"SELECT status FROM tbl_game WHERE id = '1'");
 $row = mysqli_fetch_array($query,MYSQLI_ASSOC);
 
 echo $row['status'];
?>