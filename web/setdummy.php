<?php 
 $link = mysqli_connect("localhost", "arjuna", "arjuna", "db_robot");

 if (!$link) {
 	echo "Error: Unable to connect to MySQL.";
 	exit;
 }
 else {
	echo "Berhasil";
 }
 
 $dummy1 = $_POST['dummy1'];
 $dummy2 = $_POST['dummy2'];
 $kiper = $_POST['kiper'];
 
 mysqli_query($link,"UPDATE tbl_game set dummy1 = '$dummy1', dummy2 = '$dummy2', kiper = '$kiper' where id = '1'");
 header("Location: index.php");
 exit();
?>