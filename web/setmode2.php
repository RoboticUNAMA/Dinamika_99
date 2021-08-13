<?php 
 $link = mysqli_connect("localhost", "arjuna", "arjuna", "db_robot");

 if (!$link) {
 	echo "Error: Unable to connect to MySQL.";
 	exit;
 }
 else {
	echo "Berhasil";
 }
 
 mysqli_query($link,"UPDATE tbl_game set mode = 'KICKOFF KIRI' where id = '1'");
 header("Location: index.php");
 exit();
?>